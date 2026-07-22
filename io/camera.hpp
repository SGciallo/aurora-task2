#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

// ────────────────────────────────────────────────────────────────────────────
// io（输入输出）命名空间：所有和硬件打交道的代码都在这
// ────────────────────────────────────────────────────────────────────────────
namespace io
{
/**
 * @brief 相机的"抽象接口"（基类）
 *
 * 所有具体相机（Galaxy、HikRobot、MindVision、USB Camera）都继承这个类，
 * 实现 read() 方法。这样做的好处是：换相机时只需改 YAML 配置，不用改主程序。
 *
 * 这叫做"多态"——用统一的方式操作不同的相机。
 */
class CameraBase
{
public:
  virtual ~CameraBase() = default;

  /**
   * @brief 从相机读一帧画面
   * @param img       输出：OpenCV 图像（BGR 格式，3通道彩色）
   * @param timestamp 输出：这一帧的精确时间戳（用于后续跟踪预测）
   */
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
};

/**
 * @brief 相机外壳类（"工厂模式"的包装）
 *
 * 你只需要创建 Camera 对象，它内部根据 YAML 配置自动选择用哪个相机。
 * 类似于遥控器——你只管按按钮，里面用什么协议通信你不用管。
 *
 * 用法：
 * @code
 *   io::Camera cam("configs/sentry.yaml");
 *   cv::Mat img;
 *   std::chrono::steady_clock::time_point t;
 *   cam.read(img, t);
 * @endcode
 */
class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  std::unique_ptr<CameraBase> camera_;  // 实际干活的相机对象（Galaxy/HikRobot/... 之一）
};

}  // namespace io

#endif  // IO__CAMERA_HPP
