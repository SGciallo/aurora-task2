/**
 * @file camera.cpp
 * @brief 相机驱动外壳——大恒 Galaxy（小云台项目只保留 Galaxy）
 */

#include "camera.hpp"

#include <stdexcept>

#include "galaxy/galaxy.hpp"
#include "tools/yaml.hpp"

namespace io {

Camera::Camera(const std::string &config_path) {
  auto yaml = tools::load(config_path);
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  auto gain = tools::read<double>(yaml, "gain");
  auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
  camera_ = std::make_unique<Galaxy>(exposure_ms, gain, vid_pid);
}

/**
 * @brief 读一帧画面（直接转发给底层相机驱动）
 *
 * @param img       [输出] BGR 三通道彩色图像（cv::Mat）
 * @param timestamp [输出] 该帧被采集时的精确时间戳
 *
 * 注意：read() 会阻塞，直到相机采集到新一帧为止。
 * 相机内部有一个独立的"采图线程"持续不断地从硬件抓帧，
 * read() 只是从缓冲区取最新一帧。
 */
void Camera::read(cv::Mat &img,
                  std::chrono::steady_clock::time_point &timestamp) {
  camera_->read(img, timestamp);
}

} // namespace io
