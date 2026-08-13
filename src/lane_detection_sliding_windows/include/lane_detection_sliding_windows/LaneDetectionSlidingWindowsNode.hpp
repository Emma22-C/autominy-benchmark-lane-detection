// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_DETECTION_SLIDING_WINDOWS__LANE_DETECTION_SLIDING_WINDOWS_NODE_HPP_
#define LANE_DETECTION_SLIDING_WINDOWS__LANE_DETECTION_SLIDING_WINDOWS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "lane_common/Preprocessor.hpp"
#include "lane_detection_sliding_windows/SlidingWindows.hpp"
#include "lane_detection_sliding_windows/PolynomialRegression.hpp"

#include <memory>

namespace lane_detection_sliding_windows
{

/**
 * @brief Nodo ROS2 del Sliding Windows.
 *
 * Comparte el mismo contrato de I/O que lane_detection_fcm:
 *   ~/center_deviation     std_msgs/Float32
 *   ~/angle_deviation      std_msgs/Float32
 *   ~/processing_time_ms   std_msgs/Float32
 *   ~/detection_status     std_msgs/UInt8
 *   ~/debug_image          sensor_msgs/Image
 *
 * Adicionalmente publica una métrica interna propia:
 *   ~/points_per_side      std_msgs/Float32  (promedio L+R, indicador
 *                                              de calidad del histograma)
 *
 * No publica xie_beni_* ni fpc_* (no aplican).
 */
class LaneDetectionSlidingWindowsNode : public rclcpp::Node
{
public:
  explicit LaneDetectionSlidingWindowsNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void declareAndLoadParameters();

  // Estima ángulo y desviación a partir de los coeficientes de las dos curvas
  void estimatePoseAndAngle(
    const PolynomialCoefficients & poly_left,
    const PolynomialCoefficients & poly_right,
    int image_cols,
    int image_rows,
    float & angle_deg,
    float & center_deviation_px,
    uint8_t & status_out,
    cv::Mat & debug_overlay);

  std::shared_ptr<lane_common::Preprocessor> preprocessor_;
  std::shared_ptr<SlidingWindows> sw_algorithm_;

  SlidingWindowsParams sw_params_;
  bool publish_debug_image_;
  int camera_center_offset_;

  // Polinomios "anteriores" para fallback cuando una iteración no detecta
  PolynomialCoefficients last_poly_left_;
  PolynomialCoefficients last_poly_right_;

  // ROS interfaces (mismo contrato que FCM)
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr center_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr points_pub_;
};

}  // namespace lane_detection_sliding_windows

#endif  // LANE_DETECTION_SLIDING_WINDOWS__LANE_DETECTION_SLIDING_WINDOWS_NODE_HPP_
