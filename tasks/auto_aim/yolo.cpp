/**
 * @file yolo.cpp
 * @brief YOLO 检测器 —— 小云台项目只保留 YOLOv8（OpenVINO）
 */

#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include "yolos/yolov8.hpp"

namespace auto_aim
{

YOLO::YOLO(const std::string & config_path, bool debug)
{
  yolo_ = std::make_unique<YOLOV8>(config_path, debug);
}


std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
