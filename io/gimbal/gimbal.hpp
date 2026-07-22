#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{
/*==============================================================================
 * GIMBAL COMMUNICATION PROTOCOL v2.1
 * ============================================================================
 * 支持两种协议，通过 YAML 的 protocol 字段切换:
 *   "gimbal" (默认): 帧头 5A A5, 帧尾 7F FE
 *   "cboard":        帧头 FF,   帧尾 0D   (哨兵CBoard海天协议)
 *
 * Serial Config:
 *   - BaudRate: 115200 bps
 *   - Total Timeout: 50ms
 *   - Inter-byte Timeout: 5ms
 *
 * Features:
 *   - 自动检测串口设备
 *   - 异步发送(tx_thread)
 *   - 批量读取(减少系统调用)
 *   - 数据合理性校验
 *   - 诊断信息
 * ==============================================================================
 */

// ──────────────────────── 协议类型 ────────────────────────
enum class Protocol {
  GIMBAL,   // 云台协议 (5A A5 / 7F FE)
  CBOARD    // 哨兵CBoard海天协议 (FF / 0D)
};

// ──────────────────────── 云台(Gimbal)协议 ────────────────────────
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {0x5A, 0xA5};
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];    // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
  uint8_t tail[2]= {0x7F, 0xFE};
};//43字节【云台→视觉】

static_assert(sizeof(GimbalToVision) <= 64);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {0x5A, 0xA5};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint8_t tail[2]= {0x7F, 0xFE};
};//29字节【视觉→云台】

static_assert(sizeof(VisionToGimbal) <= 64);

// ──────────────────────── CBoard(海天)协议 ────────────────────────
// 对应固件 USB_AUTO_SEND_TO_NUC_DATA_t: CBoard→视觉, 42字节
struct __attribute__((packed)) CBoardToVision {
  uint8_t head = 0xFF;
  uint8_t mode;
  float q[4];
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
  uint8_t crc8;
  uint8_t tail = 0x0D;
};
static_assert(sizeof(CBoardToVision) == 42);

// 对应固件 USB_CTRL: 视觉→CBoard, 28字节
struct __attribute__((packed)) VisionToCBoard {
  uint8_t head = 0xFF;
  uint8_t mode;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint8_t crc8;
  uint8_t tail = 0x0D;
};
static_assert(sizeof(VisionToCBoard) == 28);

enum class GimbalMode
{
  IDLE,
  AUTO_AIM,
  SMALL_BUFF,
  BIG_BUFF
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

struct GimbalDiagnostics
{
  uint32_t frame_count = 0;
  uint32_t error_count = 0;
  double rx_frame_rate = 0.0;
  double rx_latency_ms = 0.0;
  double tx_latency_ms = 0.0;
  int queue_depth = 0;
  bool serial_ok = false;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  GimbalDiagnostics diagnostics() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

private:
  serial::Serial serial_;

  std::thread thread_;
  std::thread tx_thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;
  mutable std::mutex serial_mutex_;

  // 协议选择
  Protocol protocol_ = Protocol::GIMBAL;

  // Gimbal协议数据
  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  // CBoard协议数据
  CBoardToVision rx_cboard_;
  VisionToCBoard tx_cboard_;

  // 通用发送队列（存 VisionToGimbal，CBoard 模式时字段含义相同）
  tools::ThreadSafeQueue<VisionToGimbal, true> tx_queue_{128};

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>,true>
    queue_{5000};

  std::atomic<std::int64_t> last_rx_ok_ns_{0};

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void tx_thread();
  void reconnect();
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
