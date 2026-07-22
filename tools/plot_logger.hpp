#ifndef TOOLS__PLOT_LOGGER_HPP
#define TOOLS__PLOT_LOGGER_HPP

#include <chrono>
#include <fstream>
#include <iomanip>
#include <string>

namespace tools
{

/**
 * @brief PlotJuggler 兼容的 CSV 数据记录器
 *
 * 用法:
 *   1. 程序运行时实时写入 CSV
 *   2. 打开 PlotJuggler → Data → Load CSV → 选择生成的 .csv 文件
 *   3. 勾选 "Auto reload" 即可实时看到曲线更新
 *
 * CSV 列:
 *   timestamp, gimbal_yaw, gimbal_pitch, target_yaw, target_pitch,
 *   cmd_yaw, cmd_pitch, px_off_x, px_off_y, has_target, state
 *
 * 所有角度单位为 度(deg)，像素偏移单位为 像素(px)
 */
class PlotLogger
{
public:
  PlotLogger(const std::string & filepath = "tracking_data.csv")
  {
    file_.open(filepath, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) return;

    // ── CSV 表头 ──
    file_ << "timestamp,"
          << "gimbal_yaw_deg,gimbal_pitch_deg,"
          << "target_yaw_deg,target_pitch_deg,"
          << "cmd_yaw_deg,cmd_pitch_deg,"
          << "px_off_x,px_off_y,"
          << "has_target,state\n";
    file_.flush();
    start_time_ = std::chrono::steady_clock::now();
  }

  ~PlotLogger()
  {
    if (file_.is_open()) file_.close();
  }

  /// 每帧调用一次，记录当前跟踪状态
  void log(
    float gimbal_yaw_rad, float gimbal_pitch_rad,
    float target_yaw_rad, float target_pitch_rad,
    float cmd_yaw_rad, float cmd_pitch_rad,
    float px_off_x, float px_off_y,
    bool has_target, const std::string & state)
  {
    if (!file_.is_open()) return;

    constexpr float RAD2DEG = 57.295779513f;
    double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time_).count();

    file_ << std::fixed << std::setprecision(4)
          << elapsed << ','
          << gimbal_yaw_rad * RAD2DEG << ',' << gimbal_pitch_rad * RAD2DEG << ','
          << target_yaw_rad * RAD2DEG << ',' << target_pitch_rad * RAD2DEG << ','
          << cmd_yaw_rad * RAD2DEG << ',' << cmd_pitch_rad * RAD2DEG << ','
          << px_off_x << ',' << px_off_y << ','
          << (has_target ? 1 : 0) << ','
          << state << '\n';

    // 每10帧刷盘，保证 PlotJuggler 能读到最新数据
    static int flush_counter = 0;
    if (++flush_counter >= 10) {
      file_.flush();
      flush_counter = 0;
    }
  }

  /// 刷盘（退出前调用确保数据完整）
  void flush()
  {
    if (file_.is_open()) file_.flush();
  }

private:
  std::ofstream file_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace tools

#endif  // TOOLS__PLOT_LOGGER_HPP
