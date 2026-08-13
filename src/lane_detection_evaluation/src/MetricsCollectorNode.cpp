// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_evaluation/MetricsCollectorNode.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace lane_detection_evaluation
{

MetricsCollectorNode::MetricsCollectorNode(const rclcpp::NodeOptions & options)
: Node("metrics_collector", options),
  start_wall_time_(this->now())
{
  algo_label_ = this->declare_parameter<std::string>("algo_label", "unknown");
  output_csv_path_ = this->declare_parameter<std::string>(
    "output_csv_path", "/tmp/lane_detection_eval.csv");
  detection_ns_ = this->declare_parameter<std::string>(
    "detection_namespace", "/lane_detection_fcm");
  odom_topic_ = this->declare_parameter<std::string>(
    "odom_topic", "/simulation/odom_ground_truth");
  log_fuzzy_metrics_ = this->declare_parameter<bool>("log_fuzzy_metrics", true);

  // Abrir CSV
  csv_.open(output_csv_path_, std::ios::out | std::ios::trunc);
  if (!csv_.is_open()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Could not open CSV at %s; aborting.", output_csv_path_.c_str());
    throw std::runtime_error("CSV open failed");
  }
  csv_ << "t_sec,algo,status,center_deviation_px,angle_deg,time_ms,"
       << "gt_yaw_deg,gt_x_m,gt_y_m,angle_error_deg,"
       << "xb_left,xb_right,fpc_left,fpc_right\n";

  auto qos = rclcpp::SystemDefaultsQoS();
  auto qos_sensor = rclcpp::SensorDataQoS();

  // Suscripciones a los topics estandarizados de la detección
  sub_center_ = this->create_subscription<std_msgs::msg::Float32>(
    detection_ns_ + "/center_deviation", qos,
    std::bind(&MetricsCollectorNode::onCenterDev, this, std::placeholders::_1));
  sub_angle_ = this->create_subscription<std_msgs::msg::Float32>(
    detection_ns_ + "/angle_deviation", qos,
    std::bind(&MetricsCollectorNode::onAngleDev, this, std::placeholders::_1));
  sub_time_ = this->create_subscription<std_msgs::msg::Float32>(
    detection_ns_ + "/processing_time_ms", qos,
    std::bind(&MetricsCollectorNode::onTime, this, std::placeholders::_1));
  sub_status_ = this->create_subscription<std_msgs::msg::UInt8>(
    detection_ns_ + "/detection_status", qos,
    std::bind(&MetricsCollectorNode::onStatus, this, std::placeholders::_1));

  if (log_fuzzy_metrics_) {
    sub_xb_left_ = this->create_subscription<std_msgs::msg::Float32>(
      detection_ns_ + "/xie_beni_left", qos,
      std::bind(&MetricsCollectorNode::onXbLeft, this, std::placeholders::_1));
    sub_xb_right_ = this->create_subscription<std_msgs::msg::Float32>(
      detection_ns_ + "/xie_beni_right", qos,
      std::bind(&MetricsCollectorNode::onXbRight, this, std::placeholders::_1));
    sub_fpc_left_ = this->create_subscription<std_msgs::msg::Float32>(
      detection_ns_ + "/fpc_left", qos,
      std::bind(&MetricsCollectorNode::onFpcLeft, this, std::placeholders::_1));
    sub_fpc_right_ = this->create_subscription<std_msgs::msg::Float32>(
      detection_ns_ + "/fpc_right", qos,
      std::bind(&MetricsCollectorNode::onFpcRight, this, std::placeholders::_1));
  }

  // Ground truth
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, qos_sensor,
    std::bind(&MetricsCollectorNode::onOdom, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "MetricsCollector started. algo='%s' csv='%s' detection_ns='%s' odom='%s'",
    algo_label_.c_str(), output_csv_path_.c_str(),
    detection_ns_.c_str(), odom_topic_.c_str());
}

MetricsCollectorNode::~MetricsCollectorNode()
{
  if (csv_.is_open()) {
    csv_.close();
    RCLCPP_INFO(
      this->get_logger(),
      "MetricsCollector closed. Wrote %ld rows to %s",
      row_count_, output_csv_path_.c_str());
  }
}

void MetricsCollectorNode::onCenterDev(const std_msgs::msg::Float32::SharedPtr msg)
{
  current_.stamp = this->now();
  current_.center_deviation = msg->data;
  current_.has_center = true;
  tryFlushRow();
}

void MetricsCollectorNode::onAngleDev(const std_msgs::msg::Float32::SharedPtr msg)
{
  current_.angle_deviation = msg->data;
  current_.has_angle = true;
  tryFlushRow();
}

void MetricsCollectorNode::onTime(const std_msgs::msg::Float32::SharedPtr msg)
{
  current_.time_ms = msg->data;
  current_.has_time = true;
  tryFlushRow();
}

void MetricsCollectorNode::onStatus(const std_msgs::msg::UInt8::SharedPtr msg)
{
  current_.status = msg->data;
  current_.has_status = true;
  tryFlushRow();
}

void MetricsCollectorNode::onXbLeft(const std_msgs::msg::Float32::SharedPtr msg)
{
  current_.xb_left = msg->data;
}

void MetricsCollectorNode::onXbRight(const std_msgs::msg::Float32::SharedPtr msg)
{
  current_.xb_right = msg->data;
}

void MetricsCollectorNode::onFpcLeft(const std_msgs::msg::Float32::SharedPtr msg)
{
  current_.fpc_left = msg->data;
}

void MetricsCollectorNode::onFpcRight(const std_msgs::msg::Float32::SharedPtr msg)
{
  current_.fpc_right = msg->data;
}

void MetricsCollectorNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_gt_.stamp = msg->header.stamp;
  last_gt_.x_m = msg->pose.pose.position.x;
  last_gt_.y_m = msg->pose.pose.position.y;

  // Yaw a partir del quaternion
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  last_gt_.yaw_deg = yaw * 180.0 / M_PI;
  last_gt_.valid = true;
}

void MetricsCollectorNode::tryFlushRow()
{
  // Esperamos a tener los 4 campos mínimos (center, angle, time, status)
  // y ground truth disponible.
  if (!current_.has_center || !current_.has_angle ||
    !current_.has_time || !current_.has_status)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(csv_mutex_);

  const double t_sec =
    (this->now() - start_wall_time_).seconds();

  // Para "angle_error_deg" reportamos la diferencia entre el ángulo
  // estimado (relativo al carril) y el yaw del coche en world frame.
  // OJO: esta no es una equivalencia 1-a-1 porque el ángulo del nodo
  // está expresado relativo a la imagen IPM, no en marco mundo.
  // El script post-procesado calcula la métrica final adecuadamente.
  // Aquí dejamos el dato crudo.
  const double angle_error =
    last_gt_.valid ?
    (static_cast<double>(current_.angle_deviation) - 90.0) :  // 90° = recto
    std::nan("");

  auto fmt = [](double v) -> std::string {
      if (std::isnan(v)) {return "nan";}
      std::ostringstream s;
      s << std::setprecision(6) << v;
      return s.str();
    };

  csv_ << std::fixed << std::setprecision(6)
       << t_sec << ","
       << algo_label_ << ","
       << static_cast<int>(current_.status) << ","
       << current_.center_deviation << ","
       << current_.angle_deviation << ","
       << current_.time_ms << ","
       << (last_gt_.valid ? fmt(last_gt_.yaw_deg) : "nan") << ","
       << (last_gt_.valid ? fmt(last_gt_.x_m) : "nan") << ","
       << (last_gt_.valid ? fmt(last_gt_.y_m) : "nan") << ","
       << fmt(angle_error) << ","
       << fmt(current_.xb_left.value_or(std::nan(""))) << ","
       << fmt(current_.xb_right.value_or(std::nan(""))) << ","
       << fmt(current_.fpc_left.value_or(std::nan(""))) << ","
       << fmt(current_.fpc_right.value_or(std::nan("")))
       << "\n";
  csv_.flush();

  ++row_count_;

  // Reset solo de los flags y los fuzzy metrics; mantenemos status base.
  current_.has_center = false;
  current_.has_angle = false;
  current_.has_time = false;
  current_.has_status = false;
  current_.xb_left.reset();
  current_.xb_right.reset();
  current_.fpc_left.reset();
  current_.fpc_right.reset();
}

}  // namespace lane_detection_evaluation
