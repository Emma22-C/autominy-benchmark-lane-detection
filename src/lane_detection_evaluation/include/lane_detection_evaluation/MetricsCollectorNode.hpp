// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_DETECTION_EVALUATION__METRICS_COLLECTOR_NODE_HPP_
#define LANE_DETECTION_EVALUATION__METRICS_COLLECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <fstream>
#include <mutex>
#include <string>
#include <optional>

namespace lane_detection_evaluation
{

/**
 * @brief Nodo de recolección de métricas vs ground truth.
 *
 * Se suscribe a los topics estandarizados de un nodo de detección
 * (FCM o Sliding Windows) y al ground truth del simulador
 * (/simulation/odom_ground_truth), y registra fila a fila en un CSV:
 *
 *   t_sec, algo, status, center_deviation_px, angle_deg, time_ms,
 *   gt_yaw_deg, gt_lateral_offset_m, angle_error_deg, lateral_error_m,
 *   xb_left, xb_right, fpc_left, fpc_right
 *
 * Las columnas xb_* y fpc_* solo se llenan para el FCM (NaN para SW).
 *
 * El ground truth se calcula a partir del yaw del coche en world frame
 * vs. el yaw "ideal" del carril en la pose actual del coche. Para la
 * versión inicial usamos solo el yaw del coche como referencia
 * (el revisor puede pedir ground truth de mapa más adelante).
 */
class MetricsCollectorNode : public rclcpp::Node
{
public:
  explicit MetricsCollectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MetricsCollectorNode() override;

private:
  // Datos recibidos del nodo de detección, agrupados por timestamp del bus.
  struct DetectionSnapshot
  {
    rclcpp::Time stamp;
    float center_deviation = 0.0f;
    float angle_deviation = 0.0f;
    float time_ms = 0.0f;
    uint8_t status = 0;
    std::optional<float> xb_left;
    std::optional<float> xb_right;
    std::optional<float> fpc_left;
    std::optional<float> fpc_right;

    bool has_center = false;
    bool has_angle = false;
    bool has_time = false;
    bool has_status = false;
  };

  struct GroundTruthSnapshot
  {
    rclcpp::Time stamp;
    double yaw_deg = 0.0;
    double x_m = 0.0;
    double y_m = 0.0;
    bool valid = false;
  };

  // Callbacks
  void onCenterDev(const std_msgs::msg::Float32::SharedPtr msg);
  void onAngleDev(const std_msgs::msg::Float32::SharedPtr msg);
  void onTime(const std_msgs::msg::Float32::SharedPtr msg);
  void onStatus(const std_msgs::msg::UInt8::SharedPtr msg);
  void onXbLeft(const std_msgs::msg::Float32::SharedPtr msg);
  void onXbRight(const std_msgs::msg::Float32::SharedPtr msg);
  void onFpcLeft(const std_msgs::msg::Float32::SharedPtr msg);
  void onFpcRight(const std_msgs::msg::Float32::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  void tryFlushRow();

  // Parámetros
  std::string algo_label_;
  std::string output_csv_path_;
  std::string detection_ns_;
  std::string odom_topic_;
  bool log_fuzzy_metrics_;

  // Estado
  DetectionSnapshot current_;
  GroundTruthSnapshot last_gt_;
  std::ofstream csv_;
  std::mutex csv_mutex_;
  int64_t row_count_ = 0;
  rclcpp::Time start_wall_time_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_center_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_angle_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_time_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_status_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_xb_left_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_xb_right_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_fpc_left_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_fpc_right_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

}  // namespace lane_detection_evaluation

#endif  // LANE_DETECTION_EVALUATION__METRICS_COLLECTOR_NODE_HPP_
