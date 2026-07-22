/**
 * @file math_tools.cpp
 * @brief 数学工具函数库 —— 角度转换、坐标系变换、时间计算
 *
 * 这些函数被项目所有其他模块频繁调用，是最底层的基础设施。
 *
 * ┌──────────────────── 核心函数一览 ────────────────────┐
 * │                                                  │
 * │  limit_rad()       角度归一化到 (-PI, PI]          │
 * │  eulers()          四元数/旋转矩阵 → 欧拉角        │
 * │  rotation_matrix() 欧拉角 → 旋转矩阵               │
 * │  xyz2ypd()         直角坐标 → 球坐标(yaw/pitch/距离)│
 * │  ypd2xyz()         球坐标 → 直角坐标               │
 * │  delta_time()      时间差(秒)                      │
 * │  get_abs_angle()   两向量夹角                      │
 * │                                                  │
 * └──────────────────────────────────────────────────┘
 *
 * 常用单位约定:
 *   - 角度: 弧度制（不是度数!）
 *   - 距离: 米
 *   - 时间: 秒
 *   - 欧拉角顺序: ZYX (yaw→pitch→roll), 内旋
 */

#include "math_tools.hpp"

#include <cmath>
#include <opencv2/core.hpp>  // CV_PI = 3.14159...

namespace tools
{

/**
 * @brief 将角度归一化到 (-PI, PI] 范围
 *
 * 例如: 200° → -160°,  400° → 40°,  -350° → 10°
 *
 * 这是自瞄系统里使用频率最高的函数。
 * 当你算出一个角度后，必须调用这个函数规范化，
 * 否则云台可能选择转一圈而不是转半圈到达目标。
 *
 * @param angle 任意角度（弧度）
 * @return      归一化后的角度，在 (-PI, PI] 内
 */
double limit_rad(double angle)
{
  while (angle > CV_PI) angle -= 2 * CV_PI;    // 大于180° 就减360°
  while (angle <= -CV_PI) angle += 2 * CV_PI;  // 小于等于-180° 就加360°
  return angle;
}


/**
 * @brief 四元数 → 欧拉角（通用版本，支持任意旋转顺序）
 *
 * 算法来自: https://github.com/evbernardes/quaternion_to_euler
 *
 * @param q         四元数 (w, x, y, z)
 * @param axis0     第一个旋转轴 (0=X, 1=Y, 2=Z)
 * @param axis1     第二个旋转轴
 * @param axis2     第三个旋转轴
 * @param extrinsic 是否外旋 (默认false=内旋)
 * @return          [角度0, 角度1, 角度2]，单位弧度
 *
 * 举例：eulers(q, 2, 1, 0) 表示 ZYX 顺序（最常用）
 *   先绕 Z(yaw) → 再绕 Y(pitch) → 最后绕 X(roll)
 */
Eigen::Vector3d eulers(Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic)
{
  // 内旋 ↔ 外旋 转换：反转轴顺序即可
  if (!extrinsic) std::swap(axis0, axis2);

  auto i = axis0, j = axis1, k = axis2;
  auto is_proper = (i == k);          // 首尾轴相同 = proper Euler angles (如 ZXZ, XYX)
  if (is_proper) k = 3 - i - j;       // 补全缺失的第三个轴
  auto sign = (i - j) * (j - k) * (k - i) / 2;  // 旋向符号 (+1 或 -1)

  // 四元数分量 (w, x, y, z) → 存储顺序是 (x, y, z, w)
  double a, b, c, d;
  Eigen::Vector4d xyzw = q.coeffs();
  if (is_proper) {
    a = xyzw[3];      // w
    b = xyzw[i];      // 第一轴分量
    c = xyzw[j];      // 第二轴分量
    d = xyzw[k] * sign;
  } else {
    // Tait-Bryan angles (如 ZYX, XYZ)
    a = xyzw[3] - xyzw[j];
    b = xyzw[i] + xyzw[k] * sign;
    c = xyzw[j] + xyzw[3];
    d = xyzw[k] * sign - xyzw[i];
  }

  Eigen::Vector3d eulers;

  // 第二个角度 = acos(2(a²+b²)/n² - 1)
  auto n2 = a * a + b * b + c * c + d * d;
  eulers[1] = std::acos(2 * (a * a + b * b) / n2 - 1);

  auto half_sum  = std::atan2(b, a);
  auto half_diff = std::atan2(-d, c);

  // 万向节锁检查：当第二个角接近 0 或 PI 时，第一和第三个角无法区分
  auto eps = 1e-7;
  auto safe1 = std::abs(eulers[1]) >= eps;             // 不接近0
  auto safe2 = std::abs(eulers[1] - CV_PI) >= eps;     // 不接近180°
  auto safe = safe1 && safe2;
  if (safe) {
    eulers[0] = half_sum + half_diff;    // 第一角 = 和
    eulers[2] = half_sum - half_diff;    // 第三角 = 差
  } else {
    // 万向节锁情况：强制第三角为0，只保留一个
    if (!extrinsic) {
      eulers[0] = 0;
      if (!safe1) eulers[2] = 2 * half_sum;
      if (!safe2) eulers[2] = -2 * half_diff;
    } else {
      eulers[2] = 0;
      if (!safe1) eulers[0] = 2 * half_sum;
      if (!safe2) eulers[0] = 2 * half_diff;
    }
  }

  // 归一化到 (-PI, PI]
  for (int i = 0; i < 3; i++) eulers[i] = limit_rad(eulers[i]);

  // Tait-Bryan 需要额外修正
  if (!is_proper) {
    eulers[2] *= sign;
    eulers[1] -= CV_PI / 2;
  }

  if (!extrinsic) std::swap(eulers[0], eulers[2]);

  return eulers;
}


/**
 * @brief 旋转矩阵 → 欧拉角（先转四元数再调上面的函数）
 *
 * @param R         3x3 旋转矩阵
 * @param axis0/1/2 旋转轴顺序
 * @return          欧拉角 [angle0, angle1, angle2]
 */
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic)
{
  Eigen::Quaterniond q(R);   // 旋转矩阵 → 四元数
  return eulers(q, axis0, axis1, axis2, extrinsic);
}


/**
 * @brief 欧拉角 → 旋转矩阵（ZYX 顺序）
 *
 * 绕 Z(Yaw) → Y(Pitch) → X(Roll)，内旋。
 * 这是机器人学里最常见的旋转顺序。
 *
 * @param ypr [yaw, pitch, roll]  单位：弧度
 * @return    3x3 旋转矩阵 R = Rz(yaw) * Ry(pitch) * Rx(roll)
 */
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr)
{
  double roll  = ypr[2];
  double pitch = ypr[1];
  double yaw   = ypr[0];

  double cos_yaw = cos(yaw),   sin_yaw   = sin(yaw);
  double cos_pitch = cos(pitch), sin_pitch = sin(pitch);
  double cos_roll = cos(roll),   sin_roll  = sin(roll);

  // 3x3 旋转矩阵（每行9个元素，按行排列）
  // R00             R01                              R02
  // R10             R11                              R12
  // R20             R21                              R22
  Eigen::Matrix3d R{
    {cos_yaw * cos_pitch, cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll},
    {sin_yaw * cos_pitch, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll},
    {         -sin_pitch,                                cos_pitch * sin_roll,                                cos_pitch * cos_roll}
  };
  return R;
}


/**
 * @brief 笛卡尔坐标(XYZ) → 球坐标(距离+方位角+俯仰角)
 *
 *           Z (上)
 *           │   yaw = atan2(Y, X)     水平角
 *           │   pitch = atan2(Z, sqrt(X²+Y²))  垂直角
 *           │   distance = sqrt(X²+Y²+Z²)      距离
 *           └──────→ Y (右)
 *          /
 *         X (前)
 *
 * @param xyz [x, y, z]  直角坐标
 * @return    [yaw, pitch, distance]  球坐标
 */
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];
  auto yaw   = std::atan2(y, x);
  auto pitch = std::atan2(z, std::sqrt(x * x + y * y));
  auto distance = std::sqrt(x * x + y * y + z * z);
  return {yaw, pitch, distance};
}


/**
 * @brief xyz2ypd 的雅可比矩阵（用于卡尔曼滤波的观测模型线性化）
 *
 * 雅可比矩阵是偏导数的矩阵:
 *   J[i][j] = ∂(ypd[i]) / ∂(xyz[j])
 *
 * 这告诉卡尔曼滤波："如果 xyz 变了 0.01mm, yaw/pitch/distance 各变多少"
 */
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz)
{
  auto x = xyz[0], y = xyz[1], z = xyz[2];

  // yaw = atan2(y, x)  →  ∂yaw/∂x = -y/(x²+y²),  ∂yaw/∂y = x/(x²+y²)
  auto dyaw_dx = -y / (x * x + y * y);
  auto dyaw_dy =  x / (x * x + y * y);
  auto dyaw_dz = 0.0;

  // pitch = atan2(z, sqrt(x²+y²)) 的偏导
  auto dpitch_dx = -(x * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
  auto dpitch_dy = -(y * z) / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 1.5));
  auto dpitch_dz = 1 / ((z * z / (x * x + y * y) + 1) * std::pow((x * x + y * y), 0.5));

  // distance = sqrt(x²+y²+z²) 的偏导
  auto ddistance_dx = x / std::pow((x * x + y * y + z * z), 0.5);
  auto ddistance_dy = y / std::pow((x * x + y * y + z * z), 0.5);
  auto ddistance_dz = z / std::pow((x * x + y * y + z * z), 0.5);

  // 3x3 雅可比矩阵
  Eigen::MatrixXd J{
    {dyaw_dx, dyaw_dy, dyaw_dz},
    {dpitch_dx, dpitch_dy, dpitch_dz},
    {ddistance_dx, ddistance_dy, ddistance_dz}
  };

  return J;
}


/**
 * @brief 球坐标 → 直角坐标（xyz2ypd 的逆操作）
 *
 * X = distance × cos(pitch) × cos(yaw)
 * Y = distance × cos(pitch) × sin(yaw)
 * Z = distance × sin(pitch)
 */
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
  auto x = distance * std::cos(pitch) * std::cos(yaw);
  auto y = distance * std::cos(pitch) * std::sin(yaw);
  auto z = distance * std::sin(pitch);
  return {x, y, z};
}


/**
 * @brief ypd2xyz 的雅可比矩阵（∂xyz/∂ypd）
 */
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd)
{
  auto yaw = ypd[0], pitch = ypd[1], distance = ypd[2];
  double cos_yaw = std::cos(yaw), sin_yaw = std::sin(yaw);
  double cos_pitch = std::cos(pitch), sin_pitch = std::sin(pitch);

  auto dx_dyaw = distance * cos_pitch * -sin_yaw;
  auto dy_dyaw = distance * cos_pitch *  cos_yaw;
  auto dz_dyaw = 0.0;

  auto dx_dpitch = distance * -sin_pitch * cos_yaw;
  auto dy_dpitch = distance * -sin_pitch * sin_yaw;
  auto dz_dpitch = distance *  cos_pitch;

  auto dx_ddistance = cos_pitch * cos_yaw;
  auto dy_ddistance = cos_pitch * sin_yaw;
  auto dz_ddistance = sin_pitch;

  Eigen::MatrixXd J{
    {dx_dyaw, dx_dpitch, dx_ddistance},
    {dy_dyaw, dy_dpitch, dy_ddistance},
    {dz_dyaw, dz_dpitch, dz_ddistance}
  };

  return J;
}


/**
 * @brief 计算两个时间点之间的差值（秒）
 *
 * @param a  时间点 A
 * @param b  时间点 B
 * @return   a - b 的秒数（可为负）
 *
 * 用法: double dt = delta_time(now, last_time);
 *       // dt > 0 表示 now 在 last_time 之后
 */
double delta_time(
  const std::chrono::steady_clock::time_point & a,
  const std::chrono::steady_clock::time_point & b)
{
  std::chrono::duration<double> c = a - b;
  return c.count();
}


/**
 * @brief 计算两个二维向量之间的绝对夹角
 *
 * 返回值范围: [0, PI]
 * 如果任一向量的模长为0，返回0
 *
 * 公式: angle = acos( (v1·v2) / (|v1| × |v2|) )
 *
 * @param vec1  向量1
 * @param vec2  向量2
 * @return      夹角（弧度），0~PI
 */
double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2)
{
  if (vec1.norm() == 0. || vec2.norm() == 0.) {
    return 0.;
  }
  return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
}


/**
 * @brief 将数值限制在 [min, max] 区间内
 *
 * 相当于 clamp/constrain 函数
 */
double limit_min_max(double input, double min, double max)
{
  if (input > max) return max;
  if (input < min) return min;
  return input;
}

}  // namespace tools
