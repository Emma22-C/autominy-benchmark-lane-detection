// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_DETECTION_HOUGH__LANE_DETECTION_HOUGH_NODE_HPP_
#define LANE_DETECTION_HOUGH__LANE_DETECTION_HOUGH_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "lane_common/Preprocessor.hpp"
#include "lane_detection_hough/HoughLaneExtractor.hpp"
#include "lane_detection_hough/LineRegression.hpp"

#include <memory>

namespace lane_detection_hough
{

/**
 * @brief Nodo ROS2 del detector de carril basado en Hough.
 *
 * Mismo contrato de topics que lane_detection_fcm y
 * lane_detection_sliding_windows:
 *   ~/center_deviation     std_msgs/Float32
 *   ~/angle_deviation      std_msgs/Float32
 *   ~/processing_time_ms   std_msgs/Float32
 *   ~/detection_status     std_msgs/UInt8
 *   ~/debug_image          sensor_msgs/Image
 *
 * Métrica interna específica:
 *   ~/segments_per_side    std_msgs/Float32  (promedio L+R, indicador
 *                                              de qué tan rica fue la
 *                                              evidencia para el Hough)
 *
 * No publica xie_beni_* ni fpc_* (no aplican).
 */
class LaneDetectionHoughNode : public rclcpp::Node
{
public:
  explicit LaneDetectionHoughNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void declareAndLoadParameters();

  void estimatePoseAndAngle(
    const LineModel & line_L,
    const LineModel & line_R,
    int image_cols,
    int image_rows,
    float & angle_deg,
    float & center_deviation_px,
    uint8_t & status_out,
    cv::Mat & debug_overlay);

  std::shared_ptr<lane_common::Preprocessor> preprocessor_;
  std::shared_ptr<HoughLaneExtractor> hough_extractor_;

  HoughParams hough_params_;
  bool publish_debug_image_;
  int camera_center_offset_;
  int min_endpoints_for_regression_;

  // ROS interfaces (mismo contrato que FCM/SW)
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr center_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr segments_pub_;
};

}  // namespace lane_detection_hough

#endif  // LANE_DETECTION_HOUGH__LANE_DETECTION_HOUGH_NODE_HPP_
