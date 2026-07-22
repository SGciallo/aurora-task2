/**
 * @file solver.cpp
 * @brief PnP 位姿解算器 —— 自瞄系统最核心的数学模块
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │                  它干了什么（一句话总结）                          │
 * │  输入：4个像素坐标 (YOLO找到的装甲板4个角点)                       │
 * │  输出：装甲板在世界坐标系中的3D位置 + 角度                        │
 * │                                                             │
 * │  像素(2D) → solvePnP → 相机坐标(3D) → 手眼变换 → 云台坐标 → 世界坐标 │
 * └─────────────────────────────────────────────────────────────┘
 *
 * 坐标系层次（从传感器到世界）：
 *
 *   像素坐标系 (u, v)
 *     │ 去畸变 + 内参矩阵 K 的逆
 *     ▼
 *   归一化相机坐标系 (x', y', 1)
 *     │ ★ solvePnP 在这里干活 ★
 *     ▼
 *   相机坐标系 (X_c, Y_c, Z_c)     ← 相机看到的原始3D位置
 *     │ 手眼矩阵: R_camera2gimbal * X_c + t_camera2gimbal
 *     ▼
 *   云台坐标系 (X_g, Y_g, Z_g)    ← 相对于云台旋转中心
 *     │ 云台姿态: R_gimbal2world * X_g
 *     ▼
 *   世界坐标系 (X_w, Y_w, Z_w)    ← 绝对位置（用于弹道计算）
 *
 * 关键概念：
 *   - solvePnP: OpenCV函数，已知4个3D点(装甲板模型)和4个2D像素，求解相机位姿
 *   - 手眼标定: 相机"长"在云台上，需要知道相机在云台上的安装位置和角度
 *   - 重投影: 把算出来的3D位置投回图像上，和原始像素比较，误差越小说明算得越准
 */

#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
// ────────────────── 物理常数（装甲板实际尺寸，单位：米）──────────────────
constexpr double LIGHTBAR_LENGTH = 56e-3;     // 灯条长度 56mm
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // 大装甲板宽度 230mm
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // 小装甲板宽度 135mm

// ────────────────── 装甲板3D模型点（在装甲板自身坐标系中）──────────────────
// 坐标系原点在装甲板中心，X指向前方（面朝观察者），Y向右，Z向上
//
//        (4) ┌───┐ (1)    灯条
//            │   │          ↑ Z (上)
//            │   │          │
//        (3) └───┘ (2)      └──→ Y (右)
//                           ↗
//                          X (前, 指向你)
//
// solvePnP 就是靠这4个点的3D-2D对应关系反算相机位姿
const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0,  BIG_ARMOR_WIDTH / 2,  LIGHTBAR_LENGTH / 2},   // 右上角
  {0, -BIG_ARMOR_WIDTH / 2,  LIGHTBAR_LENGTH / 2},   // 左上角
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},   // 左下角
  {0,  BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};  // 右下角

const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0,  SMALL_ARMOR_WIDTH / 2,  LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2,  LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0,  SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};


/**
 * @brief 构造函数：从 YAML 加载标定参数
 *
 * 加载的5个关键参数（全部来自标定流程 calibration/）：
 *
 *   参数                    来源                    含义
 *   ──────────────────────  ────────────────────    ─────────────────
 *   camera_matrix           calibrate_camera        相机内参（焦距、光心）
 *   distort_coeffs          calibrate_camera        畸变系数(5个)
 *   R_camera2gimbal         calibrate_handeye       相机→云台的旋转
 *   t_camera2gimbal         calibrate_handeye       相机→云台的平移
 *   R_gimbal2imubody        YAML写死的              云台→IMU本体坐标
 *
 * 注：R_gimbal2imubody 取决于 IMU 在云台上的安装方式，默认单位阵(对角=1)
 */
Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  // 读 YAML 里的数组 → 转成 Eigen 矩阵（RowMajor=逐行存储，和YAML一致）
  auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
  R_camera2gimbal_  = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
  t_camera2gimbal_  = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

  // 内参和畸变 → OpenCV 格式（OpenCV 和 Eigen 数据结构不兼容，需要转换）
  auto camera_matrix_data  = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
  Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
  cv::eigen2cv(camera_matrix,  camera_matrix_);   // Eigen → cv::Mat
  cv::eigen2cv(distort_coeffs, distort_coeffs_);
}


Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

/**
 * @brief 更新云台→世界的旋转矩阵
 * @param q 从串口读到的云台四元数（表示 IMU 当前姿态）
 *
 * 为什么要做两步变换？
 *   IMU 的坐标系和云台的坐标系通常不一致（IMU 可能倒着装）。
 *   R_imubody2imuabs  →  IMU本体 到 IMU绝对（即q本身）
 *   然后绕 R_gimbal2imubody 转回到云台坐标系
 */
void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();  // 四元数→旋转矩阵
  //                 ↑ 转置 = 求逆(旋转矩阵的逆=转置)
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}


/**
 * @brief ★ 核心函数：solvePnP 解算装甲板的3D位置和角度 ★
 * @param armor [输入/输出] 装甲板对象（输入4个像素坐标，输出计算出的所有位置信息）
 *
 * 完整流程：
 *
 *   步骤1: solvePnP
 *     输入: 4个3D模型点(BIG_ARMOR_POINTS) + 4个对应像素点(armor.points)
 *           + 内参 + 畸变
 *     输出: rvec(旋转向量) + tvec(平移向量) → 装甲板在相机坐标系中的位姿
 *
 *   步骤2: 坐标变换
 *     相机坐标 → [手眼矩阵] → 云台坐标 → [云台姿态] → 世界坐标
 *
 *   步骤3: 旋转解析
 *     从旋转矩阵提取 yaw/pitch/roll 欧拉角
 *
 *   步骤4: yaw 角度优化 (optimize_yaw)
 *     因为装甲板很薄，solvePnP 算的 yaw 角不稳定，
 *     通过重投影误差搜索最优 yaw 角
 */
void Solver::solve(Armor & armor) const
{
  // ★ 步骤1: solvePnP — 根据视角自动选大/小装甲板模型
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;    // rvec: 旋转向量(罗德里格斯), tvec: 平移向量
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);    // IPPE: 高精度算法, 比默认的ITERATIVE更适合平面目标

  // ★ 步骤2: 坐标变换 — 相机坐标 → 云台坐标 → 世界坐标
  //   相机坐标 = tvec (solvePnP输出)
  //   云台坐标 = R_camera2gimbal * 相机坐标 + t_camera2gimbal
  //   世界坐标 = R_gimbal2world * 云台坐标
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world  = R_gimbal2world_ * armor.xyz_in_gimbal;

  // ★ 步骤3: 旋转解析
  //   rvec(旋转向量) → Rodrigues变换 → 旋转矩阵 → 欧拉角(yaw/pitch/roll)
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);

  // 一步步坐标系转换
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world  = R_gimbal2world_ * R_armor2gimbal;

  // 旋转矩阵 → 欧拉角 (ZYX顺序: yaw → pitch → roll)
  // ypr = [yaw, pitch, roll]  单位：弧度
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world  = tools::eulers(R_armor2world,  2, 1, 0);

  // xyz → ypd (yaw, pitch, distance)  极坐标形式，方便计算
  armor.ypd_in_world  = tools::xyz2ypd(armor.xyz_in_world);

  // ★ 步骤4: yaw 优化
  //   平衡步兵（装甲板3/4/5号）跳过优化，因为装甲板倾斜放，假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) return;

  optimize_yaw(armor);
}


/**
 * @brief 重投影：从3D世界坐标反算回2D像素坐标
 *
 * 用途：
 *   1. 验证标定精度（重投影点和原始检测点重合 = 标定准确）
 *   2. visualize 装甲板在图像上的位置（画红框 vs YOLO的绿框）
 *   3. yaw 角搜索（先假设一个 yaw, 重投影看误差）
 *
 * 算法：
 *   世界坐标 → [逆变换] → 相机坐标 → projectPoints → 像素坐标
 *
 * @param xyz_in_world  装甲板在世界坐标系中的位置
 * @param yaw           假设的装甲板 yaw 角
 * @param type          大/小装甲板
 * @param name          装甲板编号
 * @return              4个像素坐标（按左上、右上、右下、左下顺序）
 */
std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  // 前哨站装甲板向下倾斜15°，普通装甲板向上倾斜15°
  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // 构建旋转矩阵（先绕Y转pitch, 再绕Z转yaw）
  // 即装甲板→世界坐标系的旋转
  // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // 反变换：世界 → 云台 → 相机
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() *
    (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // 旋转矩阵 → 旋转向量（OpenCV projectPoints 需要旋转向量格式）
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // 3D → 2D 投影
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}


/**
 * @brief 前哨站专用重投影误差计算（含固定pitch假设）
 * @param armor  装甲板
 * @param pitch  假设的pitch角度
 * @return       4个点的重投影误差之和（像素）
 */
double Solver::outpost_reprojection_error(Armor armor, const double & pitch)
{
  // 先做一次标准 solve（和 solve() 一样）
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world  = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world  = R_gimbal2world_ * R_armor2gimbal;
  armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  armor.ypr_in_world  = tools::eulers(R_armor2world,  2, 1, 0);
  armor.ypd_in_world  = tools::xyz2ypd(armor.xyz_in_world);

  // 用算出的 yaw 和给定的 pitch 做自定义重投影
  auto yaw = armor.ypr_in_world[0];
  auto xyz_in_world = armor.xyz_in_world;

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };

  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() *
    (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  // 累加4个点的像素误差
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}


/**
 * @brief yaw 角优化：通过重投影误差搜索最优 yaw 角
 *
 * 为什么需要优化？
 *   装甲板是一块薄板（56mm厚 vs 135/230mm宽），solvePnP 对薄目标的
 *   深度和 yaw 角的估计很不稳定。但灯条的像素位置是精确的，
 *   所以我们可以用"重投影误差最小"来找到最合理的 yaw 角。
 *
 * 算法：穷举搜索
 *   在云台当前 yaw 角 ±70° 范围内，以 1° 为步长，
 *   对每个候选 yaw 做重投影，选误差最小的那个。
 *
 *   复杂度：O(140) 次重投影，很快。
 *
 * @param armor [输入/输出] 装甲板（修改 ypr_in_world[0] 即 yaw 角）
 */
void Solver::optimize_yaw(Armor & armor) const
{
  // 获取云台当前的 yaw 角（作为搜索中心）
  Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

  constexpr double SEARCH_RANGE = 140;  // 搜索范围 140°
  auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

  auto min_error = 1e10;
  auto best_yaw  = armor.ypr_in_world[0];

  // 穷举搜索：每1°做一次重投影，选误差最小的yaw
  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw   = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    double inclined = (i - SEARCH_RANGE / 2) * CV_PI / 180.0;
    auto error = armor_reprojection_error(armor, yaw, inclined);

    if (error < min_error) {
      min_error = error;
      best_yaw  = yaw;
    }
  }

  // 保存原始 yaw（solvePnP 直接算出来的），覆盖为优化后的
  armor.yaw_raw = armor.ypr_in_world[0];
  armor.ypr_in_world[0] = best_yaw;
}


/**
 * @brief SJTU 代价函数（交大论文的方法，备用）
 * 综合考虑像素位置误差和线段方向误差
 * inclined: 倾斜程度（yaw偏离中心越多，越依赖像素位置而非角度）
 */
double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  std::size_t size = cv_refs.size();

  // 转成 Eigen 向量方便运算
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }

  double cost = 0.;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;  // 下一个点（闭合）

    Eigen::Vector2d ref_d = refs[p] - refs[i];  // 参考线段
    Eigen::Vector2d pt_d  = pts[p]  - pts[i];   // 投影线段

    // 像素位置误差：起点差 + 长度差
    double pixel_dis =
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) / ref_d.norm();

    // 角度误差：线段方向偏差
    double angular_dis =
      ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();

    // 加权组合：接近中心时看重角度，偏离中心时看重位置
    double cost_i =
      tools::square(pixel_dis  * std::sin(inclined)) +
      tools::square(angular_dis * std::cos(inclined)) * 2.0;

    cost += std::sqrt(cost_i);
  }
  return cost;
}


/**
 * @brief 装甲板重投影误差（单个 yaw 下的）
 * @param armor    装甲板（含像素坐标和3D位置）
 * @param yaw      测试的 yaw 角
 * @param inclined 倾斜参数（给SJTU cost用）
 * @return         4个角点的重投影像素误差之和
 */
double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  // 用给定的 yaw 做重投影，得到理论像素坐标
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);

  // 计算和实际检测点的像素距离
  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);

  return error;
}


/**
 * @brief 世界坐标 → 像素坐标（用于可视化）
 *
 * 把任意世界坐标系的3D点投影到图像上。
 * 例如：画出预测的装甲板位置、弹道轨迹等。
 * 自动过滤掉在相机后方（Z<0）的点。
 */
std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
{
  // 世界 → 相机 变换
  Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec, tvec;
  cv::eigen2cv(R_world2camera, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  // 过滤：只保留在相机前方的点（Z>0）
  std::vector<cv::Point3f> valid_world_points;
  for (const auto & world_point : worldPoints) {
    Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
    Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

    if (camera_point.z() > 0) {
      valid_world_points.push_back(world_point);
    }
  }

  if (valid_world_points.empty()) {
    return std::vector<cv::Point2f>();
  }

  std::vector<cv::Point2f> pixelPoints;
  cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
  return pixelPoints;
}

}  // namespace auto_aim
