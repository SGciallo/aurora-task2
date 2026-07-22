/**
 * @file small_gimbal.cpp
 * @brief 小云台自瞄程序（简化版）
 *
 * 考核任务二方向A：从零重构桌面小云台视觉跟踪系统
 *
 * 数据流: 相机 → YOLO检测 → PnP解算 → 简化跟踪 → atan2算角度 → 串口发云台
 *
 * 和哨兵主程序(auto_aim_debug_mpc.cpp)的主要区别:
 *   1. 去掉 MPC Planner，直接用 atan2 算角度
 *   2. 简化 Tracker（EMA 平滑代替 EKF，三状态代替五状态）
 *   3. 去掉子弹弹道、发射器、PlotJuggler 等比赛才需要的功能
 *   4. 单线程，不分开控制线程和视觉线程
 */

#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
// #include "tools/plot_logger.hpp"  // PlotJuggler 日志已关闭，减少磁盘 I/O

using namespace std::chrono_literals;
using namespace auto_aim;

// ─────────────────────────────────────────────────────────────────────────────
// 简化版跟踪器: 状态机 + EMA(指数移动平均)平滑
// ─────────────────────────────────────────────────────────────────────────────
class SimpleTracker {
  enum State { LOST, DETECTING, TRACKING };

  State state_ = LOST;
  int detect_count_ = 0;   // 连续检测到目标的帧数
  int lost_count_ = 0;     // 连续丢失目标的帧数

  // 平滑后的目标角度 (用 EMA 滤波)
  double smooth_yaw_ = 0.0;
  double smooth_pitch_ = 0.0;
  double prev_smooth_yaw_ = 0.0;    // 上一帧的角度，用于估算角速度
  double prev_smooth_pitch_ = 0.0;
  double yaw_vel_ = 0.0;            // 目标角速度 (rad/s)
  double pitch_vel_ = 0.0;

  // 上一帧追踪的目标 3D 位置 (用于丢失时预测、匹配最近的装甲板)
  Eigen::Vector3d last_pos_{0, 0, 0};
  ArmorName last_name_ = ArmorName::not_armor;

  static constexpr int MIN_DETECT_COUNT = 2;   // 连续2帧就确认跟踪（自定义物体检测每帧有波动，降低门槛）
  static constexpr int MAX_LOST_COUNT = 50;    // 连续50帧丢失才放弃（约1.5秒，给云台更多时间找回目标）
  static constexpr double EMA_ALPHA = 0.35;    // EMA平滑系数（提高响应速度，跟踪快速目标）

 public:
  /**
   * @brief 每帧调用: 用 YOLO 检测结果更新跟踪状态
   * @param armors  这一帧检测到的所有装甲板 (要求已做完 PnP 解算)
   * @return true 有有效目标, false 无目标
   */
  bool update(std::list<Armor> & armors)
  {
    // ── 找和上一帧跟踪目标最匹配的装甲板 ──
    Armor * best = nullptr;
    double best_dist = 1e9;

    for (auto & a : armors) {
      if (state_ != LOST && a.name != last_name_) continue;  // 跟踪中只看同名
      double d = (a.xyz_in_gimbal - last_pos_).norm();       // 3D距离
      if (d < best_dist) {
        best_dist = d;
        best = &a;
      }
    }

    bool found = (best != nullptr);

    // ── 状态机 ──
    switch (state_) {
      case LOST:
        if (!found) return false;
        // LOST 状态下 last_pos_ 是 {0,0,0}，距离参考无意义，直接接受首个检测
        state_ = DETECTING;
        detect_count_ = 1;
        last_pos_ = best->xyz_in_gimbal;
        prev_pos_ = last_pos_;
        last_vel_ = {0, 0, 0};
        last_name_ = best->name;
        break;

      case DETECTING:
        if (found && best_dist < 10.0) {   // 宽松阈值，PnP对自定义物体可能算不准距离
          detect_count_++;
          lost_count_ = 0;
          prev_pos_ = last_pos_;
          last_pos_ = best->xyz_in_gimbal;
          last_vel_ = (last_pos_ - prev_pos_) * 33.0;
          if (detect_count_ >= MIN_DETECT_COUNT) state_ = TRACKING;
        } else {
          if (++lost_count_ >= 8) {  // 容忍8帧丢失（约250ms），YOLO自定义目标检测帧间波动大
            detect_count_ = 0;
            lost_count_ = 0;
            state_ = LOST;
          }
        }
        break;

      case TRACKING:
        if (found) {
          prev_pos_ = last_pos_;              // 保存旧位置
          last_pos_ = best->xyz_in_gimbal;
          last_vel_ = (last_pos_ - prev_pos_) * 33.0;  // 实时更新速度
          lost_count_ = 0;
        } else {
          lost_count_++;
          update_prediction();  // 丢失期间用速度外推位置 → 下游自动算出预测角度
          if (lost_count_ > MAX_LOST_COUNT) {
            state_ = LOST;
            last_vel_ = {0, 0, 0};
            return false;
          }
        }
        break;
    }

    // ── 计算目标角度 (仅当有检测时从last_pos_反算; EXTRAP时角度已由update_prediction更新) ──
    if (state_ != TRACKING || lost_count_ == 0) {
      double x = last_pos_.x();
      double y = last_pos_.y();
      double z = last_pos_.z();

      double raw_yaw = std::atan2(y, x);  // Y_gimbal=R_camera2gimbal*X_camera=相机右方向
      double raw_pitch = -std::atan2(z, std::sqrt(x * x + y * y));  // 负号: 相机Y轴朝下, 云台Z轴朝上

      // ── 诊断: 输出PnP结果（每10帧打印一次，高频便于观察pitch值）──
      static int debug_log_count = 0;
      if (debug_log_count++ % 60 == 0) {
        tools::logger()->info(
          "[PnP] xyz=({:.3f},{:.3f},{:.3f}) raw_yaw={:+.2f}deg raw_pitch={:+.2f}deg state={}",
          x, y, z, raw_yaw*57.3, raw_pitch*57.3, state_str());
      }
      // ★ 额外: 当 raw_pitch 超过1°时高亮打印，方便观察pitch是否有值
      if (std::abs(raw_pitch) > 1.0 * M_PI / 180.0 && debug_log_count % 30 == 0) {
        tools::logger()->warn(
          "[PITCH_DETECT] raw_pitch={:+.2f}deg | z={:.3f} dist={:.3f}",
          raw_pitch*57.3, z, std::sqrt(x*x + y*y));
      }

      // ── EMA 平滑 (防止 YOLO 噪点导致云台抖动) ──
      prev_smooth_yaw_ = smooth_yaw_;
      prev_smooth_pitch_ = smooth_pitch_;
      if (state_ == DETECTING && detect_count_ == 1) {
        // 第一帧直接用观测值
        smooth_yaw_ = raw_yaw;
        smooth_pitch_ = raw_pitch;
        yaw_vel_ = 0.0;
        pitch_vel_ = 0.0;
      } else {
        smooth_yaw_ = EMA_ALPHA * raw_yaw + (1 - EMA_ALPHA) * smooth_yaw_;
        smooth_pitch_ = EMA_ALPHA * raw_pitch + (1 - EMA_ALPHA) * smooth_pitch_;
        // 角速度估算: 角度变化 / 帧间隔 (约0.03s, ~33fps)
        yaw_vel_ = (smooth_yaw_ - prev_smooth_yaw_) * 33.0;
        pitch_vel_ = (smooth_pitch_ - prev_smooth_pitch_) * 33.0;
      }
    }

    return true;
  }

  /// 获取平滑后的目标 yaw 角度 (rad)
  double yaw() const { return smooth_yaw_; }
  /// 获取平滑后的目标 pitch 角度 (rad)
  double pitch() const { return smooth_pitch_; }
  /// 目标 yaw 角速度 (rad/s), 用于丢失时外推
  double yaw_vel() const { return yaw_vel_; }
  /// 目标 pitch 角速度 (rad/s)
  double pitch_vel() const { return pitch_vel_; }
  /// 是否正在跟踪目标
  bool is_tracking() const { return state_ == TRACKING || state_ == DETECTING; }
  /// 当前状态名 (用于调试输出)
  const char * state_str() const
  {
    switch (state_) {
      case LOST: return "LOST";
      case DETECTING: return "DETECTING";
      case TRACKING: return lost_count_ > 0 ? "EXTRAP" : "TRACKING";
    }
    return "?";
  }

 private:
  Eigen::Vector3d prev_pos_{0, 0, 0};  // 再上一帧位置，用于差分计算速度
  Eigen::Vector3d last_vel_{0, 0, 0};  // 目标速度 (m/s)，用于丢失时外推

  /// 丢失目标时用角度速度外推，避免3D坐标变换引入的符号和放大问题
  void update_prediction()
  {
    double dt = 0.03;
    constexpr double EXTRAP_GAIN = 0.5;  // 外推幅度衰减系数，防止一步走太远
    smooth_yaw_ += yaw_vel_ * dt * EXTRAP_GAIN;
    smooth_pitch_ += pitch_vel_ * dt * EXTRAP_GAIN;

    // ── 限幅：防止外推漂移到超出相机视场角的不合理范围 ──
    // 相机水平FOV约60°, 垂直FOV约40°, 限幅留余量
    constexpr double MAX_YAW_ANGLE = 45.0 * M_PI / 180.0;    // ±45° (yaw 相对偏角)
    constexpr double MAX_PITCH_ANGLE = 30.0 * M_PI / 180.0;  // ±30° (pitch 相对偏角)
    smooth_yaw_   = std::clamp(smooth_yaw_,   -MAX_YAW_ANGLE,   MAX_YAW_ANGLE);
    smooth_pitch_ = std::clamp(smooth_pitch_, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE);

    // 3帧(~0.1s)后开始衰减速度，比之前10帧更快，防止快速漂移
    if (lost_count_ > 3) {
      double decay = 1.0 - 0.06 * (lost_count_ - 3);  // 更快衰减
      decay = std::max(0.1, decay);                     // 最低保留10%
      yaw_vel_ *= decay;
      pitch_vel_ *= decay;
    }
  }
};


// ─────────────────────────────────────────────────────────────────────────────
// 主程序
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  // ── 解析命令行参数 ──
  cv::CommandLineParser cli(argc, argv,
    "{help h  |        | 显示帮助}"
    "{test t  |        | 自动测试模式: 启动后云台自动来回摆动，验证串口链路}"
    "{@config | configs/gimbal.yaml | 配置文件路径}"
  );
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  bool auto_test = cli.has("test");

  // ── 读配置: 敌方颜色 ──
  auto yaml = YAML::LoadFile(config_path);
  auto enemy_color = (yaml["enemy_color"].as<std::string>() == "red")
                       ? Color::red
                       : Color::blue;
  double min_conf = yaml["min_confidence"].as<double>();

  // ── 初始化硬件 ──
  tools::Exiter exiter;                      // 捕获 Ctrl+C
  tools::logger()->info("正在打开串口...");
  io::Gimbal gimbal(config_path);            // 串口 (会阻塞直到收到第一帧)
  tools::logger()->info("正在打开相机...");
  io::Camera camera(config_path);            // 相机

  // ── 初始化算法模块 ──
  tools::logger()->info("正在加载 YOLO 模型...");
  auto_aim::YOLO yolo(config_path, false);   // false = 不输出 debug 信息
  auto_aim::Solver solver(config_path);       // PnP 解算器

  // ── 简化跟踪器 ──
  SimpleTracker tracker;

  // ── 主循环变量 ──
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;
  bool test_mode = auto_test;          // 测试模式：键盘直接控制云台（--test参数自动开启）
  bool test_just_entered = false;     // 刚进入测试模式，需要同步当前云台角度
  float test_yaw = 0.0f;          // 测试模式下的目标绝对yaw (rad)
  float test_pitch = 0.0f;        // 测试模式下的目标绝对pitch (rad)
  float pixel_offset_x = 0.0f, pixel_offset_y = 0.0f;  // 目标偏离画面中心的像素距离
  constexpr float TEST_STEP = 5.0f * CV_PI / 180.0f;  // 每次按键转5度
  // tools::PlotLogger plot_logger("tracking_data.csv");  // PlotJuggler 数据记录（已关闭）
  tools::logger()->info("小云台自瞄启动完成，开始跟踪...");
  tools::logger()->info("[按键] T=切换测试模式 | WASD=云台转动 | Q/ESC=退出");

  // ═══════════════════════════════════════════════════════════════════════════
  // 主循环
  // ═══════════════════════════════════════════════════════════════════════════
  while (!exiter.exit()) {
    // ── 步骤1: 读一帧图像 ──
    camera.read(img, t);

    // ★ 调试: 保存第1帧和第50帧图像到文件，验证相机画面
    if (frame_count == 0 || frame_count == 50) {
      std::string fname = fmt::format("debug_frame_{}.jpg", frame_count);
      cv::imwrite(fname, img);
      tools::logger()->info("[DEBUG] saved camera frame to {}", fname);
    }

    if (!test_mode) {
    // ── 步骤2: YOLO 检测装甲板 ──
    auto all_armors = yolo.detect(img, frame_count);

    // ── 诊断日志: YOLO原始检测数量 ──
    if (frame_count < 5 || frame_count % 60 == 0)
      tools::logger()->info("[DIAG] YOLO raw detections: {} | enemy_color={} min_conf={:.2f}",
        all_armors.size(), COLORS[enemy_color], min_conf);

    // ── 调试: 红色框 = 所有检测结果（未过滤） ──
    for (const auto & a : all_armors) {
      if (a.points.size() < 4) continue;
      std::vector<cv::Point> pts;
      for (const auto & p : a.points) pts.emplace_back(p.x, p.y);
      cv::polylines(img, pts, true, {0, 0, 255}, 1);
      cv::putText(img,
        fmt::format("{} {} {:.2f}", COLORS[a.color], ARMOR_NAMES[a.name], a.confidence),
        pts[0], cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 0, 255}, 1);
    }

    // ── 步骤2.5: 过滤 (颜色 + 置信度) ──
    auto armors = all_armors;
    armors.remove_if([&](const Armor & a) {
      return a.color != enemy_color || a.confidence < min_conf;
    });

    // ── 诊断日志: 过滤后还剩多少 ──
    if (frame_count < 5 || frame_count % 60 == 0) {
      tools::logger()->info("[DIAG] After filter: {} armors remain (raw={})",
        armors.size(), all_armors.size());
      if (!all_armors.empty() && armors.empty()) {
        tools::logger()->warn("[DIAG] All {} raw detections were filtered out!", all_armors.size());
        for (const auto & a : all_armors) {
          tools::logger()->warn("[DIAG]   color={} conf={:.3f} (need color={} conf>={:.2f})",
            COLORS[a.color], a.confidence, COLORS[enemy_color], min_conf);
        }
      }
      for (const auto & a : armors) {
        tools::logger()->info("[DIAG]   color={} name={} type={} conf={:.2f}",
          COLORS[a.color], ARMOR_NAMES[a.name], ARMOR_TYPES[a.type], a.confidence);
      }
    }

    // ── 步骤3: PnP 解算每个装甲板的3D位置 ──
    for (auto & a : armors) {
      try {
        solver.solve(a);
      } catch (const std::exception & e) {
        tools::logger()->warn("[DIAG] PnP solve failed: {}", e.what());
      }
    }

    // ── 步骤4: 简化跟踪 ──
    // ── 诊断日志: PnP解算结果 ──
    if ((frame_count < 5 || frame_count % 60 == 0) && !armors.empty()) {
      for (const auto & a : armors) {
        tools::logger()->info("[DIAG] PnP result: xyz_gimbal=({:.3f},{:.3f},{:.3f}) dist={:.3f}m",
          a.xyz_in_gimbal.x(), a.xyz_in_gimbal.y(), a.xyz_in_gimbal.z(),
          a.xyz_in_gimbal.norm());
      }
    }
    bool has_target = tracker.update(armors);

    // ── 像素偏离量: 目标中心相对画面光心的偏移 (用于判断跟踪是否居中) ──
    // 光心来自相机标定: camera_matrix[2]≈640, camera_matrix[5]≈512
    pixel_offset_x = 0.0f; pixel_offset_y = 0.0f;
    if (!armors.empty()) {
      auto & first = armors.front();
      pixel_offset_x = first.center.x - 640.0f;
      pixel_offset_y = first.center.y - 512.0f;
      // ★★★ 关键诊断: 比较像素方向和PnP方向 ★★★
      // pixel_offset_x>0 = 目标在画面右侧，pixel_offset_y<0 = 目标在画面上方
      if (frame_count % 60 == 0) {
        const char* px_dir = (pixel_offset_x > 20) ? "RIGHT" : (pixel_offset_x < -20) ? "LEFT" : "CENTER";
        const char* py_dir = (pixel_offset_y > 20) ? "DOWN" : (pixel_offset_y < -20) ? "UP" : "CENTER";
        tools::logger()->warn(
          "[PIXEL_DIR #{:3d}] px=({:+.0f},{:+.0f}) → target is {} of image center, {} of image center",
          frame_count, pixel_offset_x, pixel_offset_y, px_dir, py_dir);
      }
    }

    // ★ EXTRAP状态: 目标短暂丢失时，用角速度外推继续追踪
    //    而不是 HOLD 停止。外推角度已在 update_prediction() 中限幅 ±45°(yaw) ±30°(pitch)
    //    这样目标跑出视野时云台能继续转动，有机会重新捕捉
    bool is_extrap = (std::string(tracker.state_str()) == "EXTRAP");
    if (is_extrap) {
      if (frame_count < 10 || frame_count % 30 == 0) {
        tools::logger()->warn(
          "[EXTRAP #{:3d}] target lost, extrapolating with yaw_vel={:+.2f}deg/s pit_vel={:+.2f}deg/s",
          frame_count, tracker.yaw_vel()*57.3, tracker.pitch_vel()*57.3);
      }
      // 不强制 HOLD，继续用外推角度追踪
    }

    // ★★★ 关键诊断: 比较pixel方向和PnP target_yaw方向是否一致 ★★★
    // 如果 pixel说右但target_yaw<0，说明R_camera2gimbal或raw_yaw公式有问题
    if (has_target && std::abs(pixel_offset_x) > 30) {
      float tgt_yaw_deg = tracker.yaw() * 57.3f;
      const char* px_lr = (pixel_offset_x > 0) ? "RIGHT" : "LEFT";
      const char* pnp_lr = (tgt_yaw_deg > 0.3f) ? "RIGHT" : (tgt_yaw_deg < -0.3f) ? "LEFT" : "CENTER";
      bool mismatch = (pixel_offset_x > 30 && tgt_yaw_deg < -0.3f) ||
                      (pixel_offset_x < -30 && tgt_yaw_deg > 0.3f);
      if (mismatch) {
        tools::logger()->error(
          "[!!! DIR_MISMATCH #{:3d}] pixel={}({:+.0f}px) vs PnP target_yaw={:+.2f}deg({})! "
          "R_camera2gimbal or raw_yaw sign is likely WRONG!",
          frame_count, px_lr, pixel_offset_x, tgt_yaw_deg, pnp_lr);
      } else if (frame_count % 60 == 0) {
        tools::logger()->info(
          "[DIR_OK #{:3d}] pixel={}({:+.0f}px) matches PnP target_yaw={:+.2f}deg({})",
          frame_count, px_lr, pixel_offset_x, tgt_yaw_deg, pnp_lr);
      }
    }

    // ── 步骤5: 发送控制指令 ──
    auto gs = gimbal.state();
    // target_yaw/pitch 是 PnP 算出的相对角度（相机到目标的偏角），
    // cmd = 当前云台角度 + 误差补偿 + 速度前馈，send() 发绝对角度
    constexpr float YAW_LIMIT   =  50.0f * float(CV_PI) / 180.0f;  // ±50°
    constexpr float PITCH_LIMIT =  35.0f * float(CV_PI) / 180.0f;  // ±35°
    constexpr float YAW_GAIN = 0.28f;     // P 增益（慢速目标不超调）
    constexpr float PITCH_GAIN = 0.55f;   // pitch 增益
    constexpr float YAW_FF   = 0.03f;     // yaw 速度前馈系数（秒），保守值防止速度噪声放大
    constexpr float PITCH_FF = 0.03f;     // pitch 速度前馈系数
    constexpr float FF_RAMP = 5.0f * float(CV_PI) / 180.0f;  // 前馈渐变区间 ±5°，偏角<5°时前馈线性减弱
    // ★ 死区: 目标偏移小于此值时保持不动，防止 YOLO 微小噪声导致云台抖动
    constexpr float DEAD_ZONE_YAW   = 2.5f * float(CV_PI) / 180.0f;   // ±2.5°
    constexpr float DEAD_ZONE_PITCH = 0.5f * float(CV_PI) / 180.0f;   // ±0.5°
    // ★ yaw方向符号: -1.0 = 目标右偏→云台右转（用户实测目标在左侧时云台往右转=反了，所以取反）
    constexpr float YAW_DIR = -1.0f;
    // ★ pitch方向: +1.0（用户实测 -1.0 是反的）
    constexpr float PIT_DIR = +1.0f;
    if (has_target) {
      float target_yaw = tracker.yaw();
      float target_pitch = tracker.pitch();

      // ── 死区检查: 两个轴都在死区内才 HOLD，只要有一个轴偏了就发追踪指令 ──
      bool yaw_in_dz  = (std::abs(target_yaw)   < DEAD_ZONE_YAW);
      bool pitch_in_dz = (std::abs(target_pitch) < DEAD_ZONE_PITCH);
      if (yaw_in_dz && pitch_in_dz) {
        has_target = false;  // 两个轴都在死区 → 完全 HOLD
        if (frame_count % 30 == 0)
          tools::logger()->info("[DEADZONE #{:3d}] both axes in deadzone | yaw={:+.2f}deg pit={:+.2f}deg",
            frame_count, target_yaw*57.3, target_pitch*57.3);
      } else if (yaw_in_dz) {
        // yaw 居中但 pitch 偏离 → 只调 pitch
        if (frame_count % 10 == 0)
          tools::logger()->info("[PITCH_ONLY #{:3d}] yaw in dz, correcting pitch: tgt_pit={:+.2f}deg",
            frame_count, target_pitch*57.3);
      } else if (pitch_in_dz) {
        // pitch 居中但 yaw 偏离 → 只调 yaw
        if (frame_count % 10 == 0)
          tools::logger()->info("[YAW_ONLY #{:3d}] pitch in dz, correcting yaw: tgt_yaw={:+.2f}deg",
            frame_count, target_yaw*57.3);
      }
    }
    if (has_target) {
      float target_yaw = tracker.yaw();
      float target_pitch = tracker.pitch();
      // ★ 速度前馈: 预判目标运动方向，提前补偿
      //   yaw_vel/pitch_vel 是逐帧差分算出的瞬时角速度 (rad/s)，YOLO 单帧跳变会导致尖峰
      //   对速度再做 EMA 平滑，滤掉尖峰后再用于前馈，防止云台突然猛甩
      static float smooth_yaw_vel_ff   = 0.0f;
      static float smooth_pitch_vel_ff = 0.0f;
      constexpr float VEL_EMA_FF = 0.25f;  // 速度平滑系数
      smooth_yaw_vel_ff   = VEL_EMA_FF * tracker.yaw_vel()     + (1 - VEL_EMA_FF) * smooth_yaw_vel_ff;
      smooth_pitch_vel_ff = VEL_EMA_FF * tracker.pitch_vel()   + (1 - VEL_EMA_FF) * smooth_pitch_vel_ff;
      float ff_yaw   = YAW_FF   * smooth_yaw_vel_ff;
      float ff_pitch = PITCH_FF * smooth_pitch_vel_ff;
      // ★ 前馈渐变: 目标越靠近画面中心，前馈越弱，防止慢速目标冲过头
      //   ff_scale: 偏角=0°→0, 偏角=FF_RAMP(5°)→1, 之间线性过渡
      float ff_scale_yaw   = std::min(1.0f, std::abs(target_yaw)   / FF_RAMP);
      float ff_scale_pitch = std::min(1.0f, std::abs(target_pitch) / FF_RAMP);
      ff_yaw   *= ff_scale_yaw;
      ff_pitch *= ff_scale_pitch;
      float cmd_yaw   = gs.yaw   + YAW_DIR * (YAW_GAIN   * target_yaw   + ff_yaw);
      float cmd_pitch = gs.pitch + PIT_DIR * (PITCH_GAIN * target_pitch + ff_pitch);
      // 钳位到固件限位范围内，防止指令超限被固件截断后云台不响应
      float cmd_yaw_clamped   = std::clamp(cmd_yaw,   -YAW_LIMIT,   YAW_LIMIT);
      float cmd_pitch_clamped = std::clamp(cmd_pitch, -PITCH_LIMIT, PITCH_LIMIT);
      gimbal.send(true, false, cmd_yaw_clamped, 0.0f, 0, cmd_pitch_clamped, 0.0f, 0);
      // ── PlotJuggler 数据记录: 跟踪模式（已关闭，减少磁盘 I/O）──
      // plot_logger.log(
      //   gs.yaw, gs.pitch,                // 云台当前角度
      //   target_yaw, target_pitch,        // 目标相对偏角
      //   cmd_yaw_clamped, cmd_pitch_clamped,  // 发送的绝对角度指令
      //   pixel_offset_x, pixel_offset_y,  // 像素偏移
      //   true, tracker.state_str());      // 有目标 + 跟踪状态
      // ★ 每15帧打印一次，减少日志量
      const char * yaw_note   = (cmd_yaw   != cmd_yaw_clamped)   ? " CLAMPED" : "";
      const char * pit_note   = (cmd_pitch != cmd_pitch_clamped) ? " CLAMPED" : "";
      // ★ 三级方向诊断: pixel → PnP → cmd
      const char * px_lr = (pixel_offset_x > 20) ? "R" : (pixel_offset_x < -20) ? "L" : "0";
      const char * pnp_lr = (target_yaw > 0.02) ? "R" : (target_yaw < -0.02) ? "L" : "0";
      float cmd_delta_yaw = (cmd_yaw_clamped - gs.yaw) * 57.3f; // cmd的yaw增量(deg)
      const char * cmd_lr = (cmd_delta_yaw > 0.1f) ? "R" : (cmd_delta_yaw < -0.1f) ? "L" : "0";
      // ★ 一致性检查: pixel和PnP方向应一致
      bool px_pnp_ok = (px_lr[0] == pnp_lr[0] || px_lr[0] == '0' || pnp_lr[0] == '0');
      const char* ok_str = px_pnp_ok ? "OK" : "??";
      if (frame_count % 60 == 0)
        tools::logger()->info(
          "[CTRL #{:3d}] {} {} | px={} pnp={} cmd={}({:+5.1f}deg) → yaw={:+6.1f}deg{} | "
          "pit tgt={:+.1f}deg → cmd={:+.1f}deg{} | "
          "vel=({:+.1f},{:+.1f})deg/s ff=({:+.1f},{:+.1f})deg scl=({:.2f},{:.2f}) | gs=({:.1f},{:.1f})deg",
          frame_count, tracker.state_str(), ok_str,
          px_lr, pnp_lr, cmd_lr, cmd_delta_yaw, cmd_yaw_clamped*57.3, yaw_note,
          target_pitch*57.3, cmd_pitch_clamped*57.3, pit_note,
          tracker.yaw_vel()*57.3, tracker.pitch_vel()*57.3,
          ff_yaw*57.3, ff_pitch*57.3,
          ff_scale_yaw, ff_scale_pitch,
          gs.yaw*57.3, gs.pitch*57.3);
    } else {
      // 没有目标时保持当前位置（也要钳位），防止云台漂移
      float hold_yaw   = std::clamp(gs.yaw,   -YAW_LIMIT,   YAW_LIMIT);
      float hold_pitch = std::clamp(gs.pitch, -PITCH_LIMIT, PITCH_LIMIT);
      gimbal.send(true, false, hold_yaw, 0.0f, 0, hold_pitch, 0.0f, 0);
      // ── PlotJuggler 数据记录: HOLD 模式（已关闭）──
      // plot_logger.log(
      //   gs.yaw, gs.pitch,     // 云台当前角度
      //   0.0f, 0.0f,           // 无目标 → 偏角为0
      //   hold_yaw, hold_pitch, // 保持当前角度
      //   pixel_offset_x, pixel_offset_y,
      //   false, tracker.state_str());
      tools::logger()->info(
        "[HOLD #{:3d}] no target, hold at yaw={:+.1f}deg pit={:+.1f}deg",
        frame_count, hold_yaw*57.3, hold_pitch*57.3);
    }

    // ── 步骤6: 可视化(正常模式) ──
    for (const auto & a : armors) {
      if (a.points.size() < 4) continue;
      std::vector<cv::Point> pts;
      for (const auto & p : a.points) pts.emplace_back(p.x, p.y);
      cv::polylines(img, pts, true, {0, 255, 0}, 2);
      auto dist = a.xyz_in_gimbal.norm();
      cv::putText(img,
        fmt::format("{} {:.1f}m", ARMOR_NAMES[a.name], dist),
        pts[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
    }

    } else {
      // ═══════════════════════════════════════════════════════════════════════
      // 测试模式: 键盘直接控制云台，不经过 YOLO / PnP / Tracker
      // test_yaw/test_pitch 就是绝对目标角度(rad)，直接发给云台
      // ═══════════════════════════════════════════════════════════════════════
      auto gs = gimbal.state();

      // 刚进入测试模式时，用当前云台角度初始化 test_yaw/test_pitch
      if (test_just_entered) {
        test_just_entered = false;
        test_yaw = gs.yaw;
        test_pitch = gs.pitch;
        tools::logger()->info("[TEST] 同步当前云台角度: yaw={:.1f}deg pitch={:.1f}deg",
          test_yaw*57.3, test_pitch*57.3);
      }

      // 自动测试: 三角波缓慢扫描 ±35°，8秒半周期（~4.4°/s，极慢确保电机跟得上）
      if (auto_test) {
        constexpr float MAX_ANGLE = 35.0f * float(CV_PI) / 180.0f;  // ±35°
        constexpr float HALF_PERIOD_SEC = 8.0f;
        float t_sec = float(frame_count) * 0.03f;
        float period = 2.0f * HALF_PERIOD_SEC;
        float phase = std::fmod(t_sec, period) / period;  // [0, 1)
        float triangle;
        if (phase < 0.5f)
          triangle = -MAX_ANGLE + 4.0f * MAX_ANGLE * phase;       // -35° → +35°
        else
          triangle = 3.0f * MAX_ANGLE - 4.0f * MAX_ANGLE * phase; // +35° → -35°
        test_yaw   = triangle;
        test_pitch = triangle * 0.3f;
      }

      // 钳位到固件限位，防止指令超限被 STM32 拒绝
      test_yaw   = std::clamp(test_yaw,   -45.0f * float(CV_PI) / 180.0f, 45.0f * float(CV_PI) / 180.0f);
      test_pitch = std::clamp(test_pitch, -30.0f * float(CV_PI) / 180.0f, 30.0f * float(CV_PI) / 180.0f);

      // test_yaw/test_pitch 直接作为绝对目标角度发送
      gimbal.send(true, false, test_yaw, 0.0f, 0, test_pitch, 0.0f, 0);
      // ── PlotJuggler 数据记录: 测试模式（已关闭）──
      // plot_logger.log(
      //   gs.yaw, gs.pitch,         // 云台当前角度
      //   test_yaw - gs.yaw, test_pitch - gs.pitch,  // 目标偏角 = 期望 - 当前
      //   test_yaw, test_pitch,     // 发送的绝对角度
      //   0.0f, 0.0f,               // 测试模式无像素偏移
      //   true, "TEST");
      if (frame_count % 10 == 0)
        tools::logger()->info(
          "[TEST #{:3d}] test_yaw={:+.1f}deg test_pit={:+.1f}deg | "
          "GIMBAL yaw={:.1f}deg pit={:.1f}deg",
          frame_count, test_yaw*57.3, test_pitch*57.3,
          gs.yaw*57.3, gs.pitch*57.3);
    }

    // 状态信息
    if (test_mode) {
      auto gs_now = gimbal.state();  // 每帧更新云台实际角度
      cv::putText(img,
        fmt::format("[TEST] cmd=({:+.1f},{:+.1f})deg real=({:+.1f},{:+.1f})deg | WASD=转 R=归零",
          test_yaw * 57.3, test_pitch * 57.3,
          gs_now.yaw * 57.3, gs_now.pitch * 57.3),
        {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 255}, 2);
    } else {
      auto gs_disp = gimbal.state();
      cv::putText(img,
        fmt::format("State: {} | tgt=({:.1f},{:.1f})deg real=({:.1f},{:.1f})deg",
          tracker.state_str(), tracker.yaw() * 57.3, tracker.pitch() * 57.3,
          gs_disp.yaw * 57.3, gs_disp.pitch * 57.3),
        {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 2);
    }

    // ── 十字准星: 相机光心位置 (640, 512) ──
    {
      int cx = 640, cy = 512;
      int cross_len = 20;
      cv::line(img, {cx - cross_len, cy}, {cx + cross_len, cy}, {0, 255, 255}, 1);  // 水平线
      cv::line(img, {cx, cy - cross_len}, {cx, cy + cross_len}, {0, 255, 255}, 1);  // 竖直线
      cv::circle(img, {cx, cy}, 5, {0, 255, 255}, 1);  // 小圆
      // 显示像素偏移
      cv::putText(img,
        fmt::format("center off: ({:+.0f}, {:+.0f})px", pixel_offset_x, pixel_offset_y),
        {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 1);
    }

    cv::imshow("Small Gimbal", img);
    auto key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;  // Q 或 ESC 退出

    // ── 测试模式键盘控制 ──
    if (key == 't' || key == 'T') {
      test_mode = !test_mode;
      if (test_mode) {
        test_just_entered = true;  // 标记需要同步，下个测试帧会读取当前云台角度
      }
      tools::logger()->info("[按键] 测试模式 = {}", test_mode ? "开启" : "关闭");
    }
    if (test_mode && !test_just_entered) {
      switch (tolower(key)) {
        case 'w': test_pitch += TEST_STEP; break;  // W = pitch增大（云台向下）
        case 's': test_pitch -= TEST_STEP; break;  // S = pitch减小（云台向上）
        case 'a': test_yaw   += TEST_STEP; break;  // A = yaw增大
        case 'd': test_yaw   -= TEST_STEP; break;  // D = yaw减小
        case 'r': test_yaw = 0.0f; test_pitch = 0.0f; break;  // R = 归零
      }
      // 限制测试范围: yaw ±45°, pitch ±30°
      test_yaw   = std::clamp(test_yaw,   -45.0f * float(CV_PI) / 180.0f, 45.0f * float(CV_PI) / 180.0f);
      test_pitch = std::clamp(test_pitch, -30.0f * float(CV_PI) / 180.0f, 30.0f * float(CV_PI) / 180.0f);
    }

    frame_count++;
  }

  // ── 清理 ──
  gimbal.send(false, false, 0, 0, 0, 0, 0, 0);  // 停转
  tools::logger()->info("小云台自瞄已退出");
  return 0;
}
