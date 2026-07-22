/**
 * @file yolo.hpp
 * @brief YOLO 检测器接口 —— 和 camera.hpp 一样的"工厂模式"
 *
 * YOLO (You Only Look Once) 是目前最主流的实时目标检测算法。
 *
 * 它把一张图片输入神经网络，一次（one shot）输出：
 *   - 所有装甲板的包围框 (bbox)
 *   - 每个框里的数字分类 (1/2/3/4/5/哨兵/前哨站/基地)
 *   - 置信度 (confidence, 0~1)
 *
 * 这个文件只是外壳。真正的计算在:
 *   - yolos/yolov5.cpp  (YOLOv5, 用 OpenVINO 在 Intel CPU 上跑)
 *   - yolos/yolov8.cpp
 *   - yolos/yolo11.cpp
 *
 * 为什么叫"工厂模式"?
 *   你不需要关心底层是 YOLOv5 还是 YOLOv8，
 *   只管 new YOLO(config_path) 然后调用 detect(img)。
 *   换模型只需改 YAML 里的 yolo_name。
 */

#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim
{
/**
 * @brief YOLO 基类（抽象接口）
 * 所有具体 YOLO 版本都继承这个，实现 detect() 方法。
 */
class YOLOBase
{
public:
  virtual ~YOLOBase() = default;

  /**
   * @brief 检测一帧图像中的所有装甲板
   * @param img        BGR图像
   * @param frame_count 帧序号（用于多线程调试）
   * @return           检测到的所有装甲板列表
   */
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  /**
   * @brief 后处理（在推理输出上做NMS、坐标转换等）
   * @param output    神经网络原始输出
   * @param bgr_img   原始图像
   * @return          装甲板列表
   */
  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

/**
 * @brief YOLO 外壳（工厂）
 * 用法: auto_aim::YOLO yolo(config_path); yolo.detect(img);
 */
class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;  // 实际干活的 YOLO 版本
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
