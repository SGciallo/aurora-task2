/**
 * @file armor.hpp
 * @brief 装甲板数据结构 —— 整个自瞄系统中最基础的数据类型
 *
 * 定义一个"装甲板"包含的所有信息:
 *   - 视觉层面: 4个角点的像素坐标、颜色、数字编号
 *   - 几何层面: 在相机/云台/世界坐标系中的位置和旋转
 *   - 元数据: 置信度、优先级、类型（大/小）
 *
 * 数据流向:
 *   YOLO/Detector 输出 Armor(points, color, name, type)
 *     → Solver 补充 xyz_in_world, ypr_in_world
 *     → Tracker 用 EKF 跟踪，变成 Target
 *     → Aimer 计算瞄准点 → Planner 生成控制量
 */

#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim
{
// ────────────────── 枚举: 颜色 ──────────────────
enum Color { red, blue, extinguish, purple };  // extinguish=熄灭, purple=紫色基地
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

// ────────────────── 枚举: 装甲板类型 ──────────────────
enum ArmorType { big, small };  // 大装甲(230mm宽) / 小装甲(135mm宽)
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

// ────────────────── 枚举: 装甲板编号 ──────────────────
// 1/2/3/4/5 = 步兵编号, sentry=哨兵, outpost=前哨站, base=基地
enum ArmorName { one, two, three, four, five, sentry, outpost, base, not_armor };
const std::vector<std::string> ARMOR_NAMES = {
  "one", "two", "three", "four", "five", "sentry", "outpost", "base", "not_armor"};

// ────────────────── 枚举: 打击优先级 ──────────────────
// 哨兵 > 英雄(3) > 步兵(4/5) > 步兵(1/2)
// first最高, fifth最低
enum ArmorPriority { first = 1, second, third, forth, fifth };

// ────────────────── 所有可能的装甲板组合（颜色×编号×类型） ──────────────────
// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},
  {blue, three, big},        {red, three, big},        {extinguish, three, big},
  {blue, four, big},         {red, four, big},         {extinguish, four, big},
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

// ────────────────── 灯条结构体 ──────────────────
// 每个装甲板由两个灯条组成（左灯条 + 右灯条）
struct Lightbar
{
  std::size_t id;
  Color color;
  cv::Point2f center, top, bottom, top2bottom;        // 几何属性: 中心/顶部/底部
  std::vector<cv::Point2f> points;                     // 灯条的4个角点
  double angle, angle_error, length, width, ratio;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() {};
};

// ────────────────── ★ 装甲板结构体（最重要的数据结构）★ ──────────────────
struct Armor
{
  // ─── 视觉层: YOLO/Detector 输出的 ───
  Color color;                    // 红色 / 蓝色
  Lightbar left, right;           // 左右灯条（传统视觉检测用）
  cv::Point2f center;             // 装甲板中心像素（不是对角线交点，是灯条连线的中点）
  cv::Point2f center_norm;        // 归一化坐标 (x/image_width, y/image_height)
  std::vector<cv::Point2f> points; // 4个角点: [左上, 右上, 右下, 左下] → 给 solvePnP 用

  // ─── 几何校验 ───
  double ratio;                   // 灯条间距 / 长灯条长度（用于判断大小装甲板）
  double side_ratio;              // 长灯条 / 短灯条
  double rectangular_error;       // 矩程度（灯条和连线是否构成矩形）

  // ─── 分类 ───
  ArmorType type;                 // 大装甲(big) / 小装甲(small)
  ArmorName name;                 // 编号: one~five, sentry, outpost, base
  ArmorPriority priority;         // 打击优先级
  int class_id;                   // 分类器输出的数字ID
  cv::Rect box;                   // 包围框
  cv::Mat pattern;                // 装甲板数字区域的图像块
  double confidence;              // 置信度 (0~1)
  bool duplicated;                // 是否重复检测

  // ─── 几何层: Solver 解算出来的 ───
  Eigen::Vector3d xyz_in_gimbal;  // 云台坐标系下的3D位置 (m)
  Eigen::Vector3d xyz_in_world;   // 世界坐标系下的3D位置 (m)
  Eigen::Vector3d ypr_in_gimbal;  // 云台坐标系下的欧拉角 [yaw, pitch, roll] (rad)
  Eigen::Vector3d ypr_in_world;   // 世界坐标系下的欧拉角 (rad)
  Eigen::Vector3d ypd_in_world;   // 球坐标系 [yaw, pitch, distance] (rad, rad, m)

  double yaw_raw;                 // solvePnP 直接输出的原始 yaw（未优化）

  // ─── 构造函数 ───
  Armor(const Lightbar & left, const Lightbar & right);  // 传统视觉用
  Armor(int class_id, float confidence, const cv::Rect & box,
        std::vector<cv::Point2f> armor_keypoints);        // YOLO 用
  Armor(int class_id, float confidence, const cv::Rect & box,
        std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
  Armor(int color_id, int num_id, float confidence, const cv::Rect & box,
        std::vector<cv::Point2f> armor_keypoints);
  Armor(int color_id, int num_id, float confidence, const cv::Rect & box,
        std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__ARMOR_HPP
