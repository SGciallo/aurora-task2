#include "gimbal.hpp"

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <vector>
#include <fmt/ranges.h>

namespace io
{
namespace {
constexpr bool kVerboseSerialHexLog = true;

std::string resolve_serial_port(const std::string & configured_port)
{
  namespace fs = std::filesystem;

  if (!configured_port.empty() && fs::exists(configured_port)) {
    return configured_port;
  }

  static const std::vector<std::string> preferred_ports = {
    "/dev/gimbal",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
  };

  for (const auto & port : preferred_ports) {
    if (fs::exists(port)) {
      return port;
    }
  }

  // 扫描 /dev 下所有 ttyUSB/ttyACM 设备
  std::vector<std::string> detected_ports;
  try {
    for (const auto & entry : fs::directory_iterator("/dev")) {
      if (!entry.is_character_file()) continue;
      const auto name = entry.path().filename().string();
      if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0) {
        detected_ports.push_back(entry.path().string());
      }
    }
  } catch (...) {
    return configured_port;
  }

  if (detected_ports.empty()) return configured_port;
  std::sort(detected_ports.begin(), detected_ports.end());
  return detected_ports.front();
}
}  // namespace

Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  // 读取协议类型 (默认gimbal)
  std::string protocol_str = "gimbal";
  if (yaml["protocol"])
    protocol_str = yaml["protocol"].as<std::string>();
  if (protocol_str == "cboard")
    protocol_ = Protocol::CBOARD;
  else
    protocol_ = Protocol::GIMBAL;

  tools::logger()->info("[Gimbal] Using protocol: {}", protocol_str);

  auto resolved_port = resolve_serial_port(com_port);
  if (resolved_port != com_port) {
    tools::logger()->warn(
      "[Gimbal] Configured com_port '{}' not found, fallback to '{}'.", com_port, resolved_port);
  }

  try {
    serial_.setPort(resolved_port);
    serial_.setBaudrate(115200);

    serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
    timeout.inter_byte_timeout = 5;
    serial_.setTimeout(timeout);

    serial_.open();

    // 初始化时清空积压数据
    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_.isOpen()) {
        serial_.flushInput();
        serial_.flushOutput();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    } catch (...) {}
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);
  tx_thread_ = std::thread(&Gimbal::tx_thread, this);

  // 等待云台发第一帧数据，最多等2秒
  // 如果固件需要先收到指令才会回复，超时后不阻塞，让主循环发指令
  {
    auto start = std::chrono::steady_clock::now();
    constexpr auto kTimeout = std::chrono::seconds(2);
    while (queue_.empty()) {
      if (std::chrono::steady_clock::now() - start > kTimeout) {
        tools::logger()->warn(
          "[Gimbal] No response from gimbal after {}s, proceeding anyway.",
          std::chrono::duration_cast<std::chrono::seconds>(kTimeout).count());
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!queue_.empty()) {
      queue_.pop();
      tools::logger()->info("[Gimbal] First q received.");
    }
  }
}

Gimbal::~Gimbal()
{
  quit_ = true;
  tx_queue_.push(VisionToGimbal{});  // 唤醒发送线程
  if (thread_.joinable()) thread_.join();
  if (tx_thread_.joinable()) tx_thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

GimbalDiagnostics Gimbal::diagnostics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  GimbalDiagnostics diag;
  diag.serial_ok = serial_.isOpen();
  auto now_ns = std::chrono::steady_clock::now().time_since_epoch().count();
  auto last_rx_ns = last_rx_ok_ns_.load();
  if (last_rx_ns > 0) {
    diag.rx_latency_ms = static_cast<double>(now_ns - last_rx_ns) / 1e6;
  }
  return diag;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:       return "IDLE";
    case GimbalMode::AUTO_AIM:   return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF: return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:   return "BIG_BUFF";
    default:                     return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;
    return q_c;
  }
}

// ──────────────────── 发送（异步，入队即返回）────────────────────
void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  // 固件把收到的值当作「绝对目标角度(deg)」来执行
  // 所以这里直接发绝对角度，不减去 gs.yaw 变成差值
  // 固件用「度」，内部用弧度，发送前转换
  constexpr float RAD2DEG = 57.295779513f;
  VisionToGimbal.yaw *= RAD2DEG;
  VisionToGimbal.yaw_vel *= RAD2DEG;
  VisionToGimbal.yaw_acc *= RAD2DEG;
  VisionToGimbal.pitch *= RAD2DEG;
  VisionToGimbal.pitch_vel *= RAD2DEG;
  VisionToGimbal.pitch_acc *= RAD2DEG;

  tx_queue_.push(VisionToGimbal);
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  // 固件把收到的值当作「绝对目标角度(deg)」，
  // 所以直接发调用者传入的绝对角度，不减去当前角度变成差值

  // 固件用「度」和「度/秒」，需要从弧度转换
  constexpr float RAD2DEG = 57.295779513f;  // 180/π
  VisionToGimbal pkt;
  pkt.mode = control ? (fire ? 2 : 1) : 0;
  pkt.yaw = yaw * RAD2DEG;
  pkt.yaw_vel = yaw_vel * RAD2DEG;
  pkt.yaw_acc = yaw_acc * RAD2DEG;
  pkt.pitch = pitch * RAD2DEG;
  pkt.pitch_vel = pitch_vel * RAD2DEG;
  pkt.pitch_acc = pitch_acc * RAD2DEG;

  tx_queue_.push(pkt);
}

// ──────────────────── 发送线程 ────────────────────
void Gimbal::tx_thread()
{
  static int tx_log_count = 0;
  while (true) {
    VisionToGimbal pkt;
    tx_queue_.pop(pkt);
    if (quit_) break;

    // 背压：只发最新控制量
    VisionToGimbal newest = pkt;
    while (!tx_queue_.empty()) {
      tx_queue_.pop(newest);
    }

    if (protocol_ == Protocol::CBOARD) {
      // ───── CBoard 发送 (28字节) ─────
      VisionToCBoard cb_pkt;
      cb_pkt.head = 0xFF;
      cb_pkt.mode = newest.mode;
      cb_pkt.yaw = newest.yaw;
      cb_pkt.yaw_vel = newest.yaw_vel;
      cb_pkt.yaw_acc = newest.yaw_acc;
      cb_pkt.pitch = newest.pitch;
      cb_pkt.pitch_vel = newest.pitch_vel;
      cb_pkt.pitch_acc = newest.pitch_acc;
      cb_pkt.crc8 = 0;
      cb_pkt.tail = 0x0D;

      if (tx_log_count++ % 100 == 0) {
      std::string hex;
      hex.reserve(sizeof(cb_pkt) * 3);
      const auto * p = reinterpret_cast<const uint8_t *>(&cb_pkt);
      for (size_t i = 0; i < sizeof(cb_pkt); ++i) {
        char b[4];
        std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(p[i]));
        hex += b;
      }
      tools::logger()->debug("[Gimbal] CBoard TX hex: {}", hex);
      uint8_t tm2 = cb_pkt.mode;
      float ty2 = cb_pkt.yaw, tp2 = cb_pkt.pitch;
      float tv2 = cb_pkt.yaw_vel, pv2 = cb_pkt.pitch_vel;
      tools::logger()->debug("[Gimbal] CBoard TX parsed: mode={} yaw={:.3f}deg({:.3f}rad) pitch={:.3f}deg({:.3f}rad) vely={:.1f}deg/s velp={:.1f}deg/s",
        tm2, ty2, ty2 / 57.29578, tp2, tp2 / 57.29578, tv2, pv2);
      }

      auto tx_start = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> serial_lock(serial_mutex_);
      try {
        serial_.write(reinterpret_cast<uint8_t *>(&cb_pkt), sizeof(cb_pkt));
        auto tx_end = std::chrono::steady_clock::now();
        double tx_latency = std::chrono::duration<double, std::milli>(tx_end - tx_start).count();
        if (tx_latency > 5.0) {
          tools::logger()->warn("[Gimbal] High TX latency: {:.2f}ms", tx_latency);
        }
      } catch (const std::exception & e) {
        tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
      }
    } else {
      // ───── Gimbal 发送 (29字节) ─────
      if (tx_log_count++ % 100 == 0) {
      std::string hex;
      hex.reserve(sizeof(newest) * 3);
      const auto * p = reinterpret_cast<const uint8_t *>(&newest);
      for (size_t i = 0; i < sizeof(newest); ++i) {
        char b[4];
        std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(p[i]));
        hex += b;
      }
      tools::logger()->debug("[Gimbal] TX hex: {}", hex);
      uint8_t tm = newest.mode;
      float ty = newest.yaw, tp = newest.pitch;
      float tv = newest.yaw_vel, pv = newest.pitch_vel;
      tools::logger()->debug("[Gimbal] TX parsed: mode={} yaw={:.3f}deg({:.3f}rad) pitch={:.3f}deg({:.3f}rad) vely={:.1f}deg/s velp={:.1f}deg/s",
        tm, ty, ty / 57.29578, tp, tp / 57.29578, tv, pv);
      }

      auto tx_start = std::chrono::steady_clock::now();
      std::lock_guard<std::mutex> serial_lock(serial_mutex_);
      try {
        serial_.write(reinterpret_cast<uint8_t *>(&newest), sizeof(newest));
        auto tx_end = std::chrono::steady_clock::now();
        double tx_latency = std::chrono::duration<double, std::milli>(tx_end - tx_start).count();
        if (tx_latency > 5.0) {
          tools::logger()->warn("[Gimbal] High TX latency: {:.2f}ms", tx_latency);
        }
      } catch (const std::exception & e) {
        tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
      }
    }
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");

  // 根据协议选择参数
  const bool is_cboard = (protocol_ == Protocol::CBOARD);
  const int PACKET_SIZE = is_cboard ? 42 : 43;
  const int FRAME_HEAD_SIZE = is_cboard ? 1 : 2;

  uint8_t buffer[64];  // 临时帧缓冲（够大）
  uint8_t stream_buf[512] = {0};
  size_t stream_len = 0;
  int error_count = 0;
  bool has_received_data = false;
  int no_data_count = 0;

  while (!quit_) {
    if (error_count > 100) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many consecutive errors (>100), attempting to reconnect...");
      reconnect();
      continue;
    }

    // 1) 批量读取
    size_t nread = 0;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      try {
        size_t avail = serial_.available();
        if (avail > 0) {
          size_t room = sizeof(stream_buf) - stream_len;
          if (room == 0) {
            std::memmove(stream_buf, stream_buf + sizeof(stream_buf) - 64, 64);
            stream_len = 64;
            room = sizeof(stream_buf) - stream_len;
          }
          size_t to_read = std::min(avail, room);
          nread = serial_.read(stream_buf + stream_len, to_read);
          stream_len += nread;
        }
      } catch (const std::exception & e) {
        tools::logger()->warn("[Gimbal] Failed to read serial stream: {}", e.what());
      }
    }

    if (nread == 0) {
      no_data_count++;
      if (no_data_count > 4000) {
        error_count++;
        no_data_count = 0;
        tools::logger()->warn("[Gimbal] No serial data for extended period, triggering reconnect");
      }
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }
    no_data_count = 0;

    // 原始RX日志：只打印前5帧，减少刷屏
    {
      static int raw_rx_count = 0;
      if (raw_rx_count++ < 5) {
        std::string hex;
        hex.reserve(nread * 3);
        const auto * p = stream_buf + stream_len - nread;
        for (size_t i = 0; i < std::min(nread, (size_t)80); ++i) {
          char b[4];
          std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(p[i]));
          hex += b;
        }
        tools::logger()->debug("[Gimbal] raw RX {} bytes: {}", nread, hex);
      }
    }

    // 2) 在流缓冲中找帧头
    if (stream_len < static_cast<size_t>(FRAME_HEAD_SIZE)) continue;

    size_t head_pos = static_cast<size_t>(-1);

    if (is_cboard) {
      // CBoard: 找 0xFF
      for (size_t i = 0; i < stream_len; ++i) {
        if (stream_buf[i] == 0xFF) {
          head_pos = i;
          break;
        }
      }
    } else {
      // Gimbal: 找 5A A5
      for (size_t i = 1; i < stream_len; ++i) {
        if (stream_buf[i - 1] == 0x5A && stream_buf[i] == 0xA5) {
          head_pos = i - 1;
          break;
        }
      }
    }

    if (head_pos == static_cast<size_t>(-1) && !is_cboard && stream_len >= static_cast<size_t>(PACKET_SIZE)) {
      // ★ 后备同步: 用 7F FE 帧尾定位帧 (解决非标波特率下 5A A5 不可靠的问题)
      for (size_t i = PACKET_SIZE - 1; i < stream_len; ++i) {
        if (stream_buf[i - 1] == 0x7F && stream_buf[i] == 0xFE) {
          // 从帧尾往前推 PACKET_SIZE 字节作为帧起始
          head_pos = i - (PACKET_SIZE - 1);
          static int tail_sync_count = 0;
          if (tail_sync_count++ < 5) {
            tools::logger()->warn(
              "[Gimbal] Fallback sync using 7FFE tail (no 5AA5 header found), head_pos={}",
              head_pos);
          }
          break;
        }
      }
    }

    if (head_pos == static_cast<size_t>(-1)) {
      // 保留最后 PACKET_SIZE 字节作为下次匹配窗口
      size_t keep = std::min(stream_len, static_cast<size_t>(PACKET_SIZE));
      if (stream_len > keep) {
        std::memmove(stream_buf, stream_buf + stream_len - keep, keep);
      }
      stream_len = keep;
      continue;
    }

    if (stream_len - head_pos < static_cast<size_t>(PACKET_SIZE)) {
      // 数据不够一帧，挪到开头等后续字节
      if (head_pos > 0) {
        std::memmove(stream_buf, stream_buf + head_pos, stream_len - head_pos);
        stream_len -= head_pos;
      }
      continue;
    }

    std::memcpy(buffer, stream_buf + head_pos, PACKET_SIZE);
    size_t consumed = head_pos + PACKET_SIZE;
    if (stream_len > consumed) {
      std::memmove(stream_buf, stream_buf + consumed, stream_len - consumed);
      stream_len -= consumed;
    } else {
      stream_len = 0;
    }

    // 3) 帧尾检查
    bool tail_ok = false;
    if (is_cboard) {
      tail_ok = (buffer[PACKET_SIZE - 1] == 0x0D);
    } else {
      tail_ok = (buffer[PACKET_SIZE - 1] == 0xFE && buffer[PACKET_SIZE - 2] == 0x7F);
    }

    if (!tail_ok) {
      error_count++;
      // 跳过帧头的第一个字节，防止把数据里的假帧头当真了
      size_t skip = (is_cboard) ? 1 : 2;
      if (stream_len >= skip) {
        std::memmove(stream_buf, stream_buf + skip, stream_len - skip);
        stream_len -= skip;
      } else {
        stream_len = 0;
      }
      continue;
    }

    if (error_count > 0) error_count--;

    // 4) 解析
    auto t = std::chrono::steady_clock::now();

    // 提取数据用于校验和存储
    float q_raw[4], yaw_val, pitch_val, yaw_vel_val, pitch_vel_val;
    float bullet_speed_val;
    uint16_t bullet_count_val;
    uint8_t mode_val;

    // 固件发过来的角度/角速度单位是「度」和「度/秒」，需要转为内部使用的弧度和弧度/秒
    constexpr float DEG2RAD = 0.01745329252f;  // π/180

    if (is_cboard) {
      memcpy(&rx_cboard_, buffer, PACKET_SIZE);
      q_raw[0] = rx_cboard_.q[0]; q_raw[1] = rx_cboard_.q[1];
      q_raw[2] = rx_cboard_.q[2]; q_raw[3] = rx_cboard_.q[3];
      yaw_val = rx_cboard_.yaw * DEG2RAD;
      pitch_val = rx_cboard_.pitch * DEG2RAD;
      yaw_vel_val = rx_cboard_.yaw_vel * DEG2RAD;
      pitch_vel_val = rx_cboard_.pitch_vel * DEG2RAD;
      bullet_speed_val = rx_cboard_.bullet_speed;
      bullet_count_val = rx_cboard_.bullet_count;
      mode_val = rx_cboard_.mode;
    } else {
      memcpy(&rx_data_, buffer, PACKET_SIZE);
      q_raw[0] = rx_data_.q[0]; q_raw[1] = rx_data_.q[1];
      q_raw[2] = rx_data_.q[2]; q_raw[3] = rx_data_.q[3];
      yaw_val = rx_data_.yaw * DEG2RAD;
      pitch_val = rx_data_.pitch * DEG2RAD;
      yaw_vel_val = rx_data_.yaw_vel * DEG2RAD;
      pitch_vel_val = rx_data_.pitch_vel * DEG2RAD;
      bullet_speed_val = rx_data_.bullet_speed;
      bullet_count_val = rx_data_.bullet_count;
      mode_val = rx_data_.mode;
    }

    // 5) 数据合理性检查（弧度，yaw可以多圈旋转，阈值放宽到±100 rad）
    const float MAX_ANGLE = 100.0f;
    const float MAX_VEL = 50.0f;
    if (std::abs(yaw_val) > MAX_ANGLE || std::abs(pitch_val) > MAX_ANGLE ||
        std::abs(yaw_vel_val) > MAX_VEL || std::abs(pitch_vel_val) > MAX_VEL) {
      error_count++;
      tools::logger()->warn(
        "[Gimbal] Data sanity check failed: yaw={:.3f}, pitch={:.3f}, yaw_vel={:.3f}, pitch_vel={:.3f}",
        yaw_val, pitch_val, yaw_vel_val, pitch_vel_val);
      continue;
    }

    if (!has_received_data) {
      std::string hex;
      hex.reserve(PACKET_SIZE * 3);
      for (int i = 0; i < PACKET_SIZE; ++i) {
        char b[4];
        std::snprintf(b, sizeof(b), "%02X ", static_cast<unsigned>(buffer[i]));
        hex += b;
      }
      tools::logger()->info("[Gimbal] First {} rx frame ({} bytes): {}",
        is_cboard ? "CBoard" : "", PACKET_SIZE, hex);
    }

    error_count = 0;
    has_received_data = true;
    Eigen::Quaterniond q(q_raw[0], q_raw[1], q_raw[2], q_raw[3]);

    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_.yaw = yaw_val;
      state_.yaw_vel = yaw_vel_val;
      state_.pitch = pitch_val;
      state_.pitch_vel = pitch_vel_val;
      state_.bullet_speed = bullet_speed_val;
      state_.bullet_count = bullet_count_val;

      switch (mode_val) {
        case 0: mode_ = GimbalMode::IDLE; break;
        case 1: mode_ = GimbalMode::AUTO_AIM; break;
        case 2: mode_ = GimbalMode::SMALL_BUFF; break;
        case 3: mode_ = GimbalMode::BIG_BUFF; break;
        default:
          mode_ = GimbalMode::IDLE;
          tools::logger()->warn("[Gimbal] Invalid mode: {}", mode_val);
          break;
      }
    }

    last_rx_ok_ns_.store(t.time_since_epoch().count());
    queue_.push({q, t});

    // 定期诊断
    static int diag_counter = 0;
    if (++diag_counter >= 200) {
      diag_counter = 0;
      auto diag = diagnostics();
      tools::logger()->info(
        "[Gimbal] Diag: latency={:.2f}ms, serial_ok={}",
        diag.rx_latency_ms,
        diag.serial_ok ? "yes" : "no");
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 15;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      if (serial_.isOpen()) {
        serial_.flushInput();
        serial_.flushOutput();
        serial_.close();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } catch (...) {}

    try {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      serial_.open();

      serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
      timeout.inter_byte_timeout = 5;
      serial_.setTimeout(timeout);

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      serial_.flushInput();
      serial_.flushOutput();

      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

}  // namespace io
