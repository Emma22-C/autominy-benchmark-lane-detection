// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_sliding_windows/LaneDetectionSlidingWindowsNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <cmath>
#include <sstream>

namespace lane_detection_sliding_windows
{

LaneDetectionSlidingWindowsNode::LaneDetectionSlidingWindowsNode(
  const rclcpp::NodeOptions & options)
: Node("lane_detection_sliding_windows", options)
{
  declareAndLoadParameters();

  // Construir el preprocessor idéntico al del FCM (parámetros del YAML)
  lane_common::PreprocessorParams pp_params;
  pp_params.resize_factor = this->get_parameter_or("preprocessing.resize_factor", 0.5);
  pp_params.gray_threshold_low = this->get_parameter_or("preprocessing.gray_threshold_low", 200);
  pp_params.gray_threshold_high = this->get_parameter_or("preprocessing.gray_threshold_high", 255);
  pp_params.canny_low = this->get_parameter_or("preprocessing.canny_low", 300);
  pp_params.canny_high = this->get_parameter_or("preprocessing.canny_high", 700);
  pp_params.canny_aperture = this->get_parameter_or("preprocessing.canny_aperture", 3);
  pp_params.median_blur_kernel = this->get_parameter_or("preprocessing.median_blur_kernel", 5);
  pp_params.roi_left_x = this->get_parameter_or("preprocessing.roi_left_x", 0);
  pp_params.roi_left_y = this->get_parameter_or("preprocessing.roi_left_y", 0);
  pp_params.roi_left_width = this->get_parameter_or("preprocessing.roi_left_width", 160);
  pp_params.roi_left_height = this->get_parameter_or("preprocessing.roi_left_height", 240);
  pp_params.roi_right_x = this->get_parameter_or("preprocessing.roi_right_x", 160);
  pp_params.roi_right_y = this->get_parameter_or("preprocessing.roi_right_y", 0);
  pp_params.roi_right_width = this->get_parameter_or("preprocessing.roi_right_width", 160);
  pp_params.roi_right_height = this->get_parameter_or("preprocessing.roi_right_height", 240);
  pp_params.apply_white_filter =
    this->get_parameter_or("preprocessing.apply_white_filter", true);
  pp_params.white_filter_max_width =
    this->get_parameter_or("preprocessing.white_filter_max_width", 20);

  {
    const auto ipm_src_flat = this->get_parameter_or<std::vector<double>>(
      "preprocessing.ipm_src_flat",
      {56.0, 110.0, 250.0, 110.0, 0.0, 200.0, 320.0, 200.0});
    const auto ipm_dst_flat = this->get_parameter_or<std::vector<double>>(
      "preprocessing.ipm_dst_flat",
      {56.0, 0.0, 250.0, 0.0, 56.0, 240.0, 250.0, 240.0});
    if (ipm_src_flat.size() == 8 && ipm_dst_flat.size() == 8) {
      for (int i = 0; i < 4; ++i) {
        pp_params.ipm_src[i] = cv::Point2f(
          static_cast<float>(ipm_src_flat[2 * i]),
          static_cast<float>(ipm_src_flat[2 * i + 1]));
        pp_params.ipm_dst[i] = cv::Point2f(
          static_cast<float>(ipm_dst_flat[2 * i]),
          static_cast<float>(ipm_dst_flat[2 * i + 1]));
      }
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "ipm_src_flat/ipm_dst_flat must have 8 elements each; using defaults.");
    }
  }

  preprocessor_ = std::make_shared<lane_common::Preprocessor>(pp_params);
  sw_algorithm_ = std::make_shared<SlidingWindows>(sw_params_);

  const std::string input_topic = this->declare_parameter<std::string>(
    "input_image_topic", "/sensors/camera/color/image_rect_color");

  auto qos = rclcpp::SensorDataQoS();
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic, qos,
    std::bind(&LaneDetectionSlidingWindowsNode::imageCallback, this, std::placeholders::_1));

  center_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/center_deviation", 10);
  angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/angle_deviation", 10);
  time_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/processing_time_ms", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("~/detection_status", 10);
  points_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/points_per_side", 10);

  if (publish_debug_image_) {
    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/debug_image", 1);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "lane_detection_sliding_windows initialized. Subscribed to: %s "
    "| n_windows=%d, margin=%d, minpix=%d",
    input_topic.c_str(),
    sw_params_.n_windows, sw_params_.margin, sw_params_.min_pixels_per_window);
}

void LaneDetectionSlidingWindowsNode::declareAndLoadParameters()
{
  sw_params_.n_windows = this->declare_parameter<int>("sliding_windows.n_windows", 9);
  sw_params_.margin = this->declare_parameter<int>("sliding_windows.margin", 20);
  sw_params_.min_pixels_per_window =
    this->declare_parameter<int>("sliding_windows.min_pixels_per_window", 40);
  sw_params_.min_points_for_regression =
    this->declare_parameter<int>("sliding_windows.min_points_for_regression", 75);

  publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", true);
  camera_center_offset_ = this->declare_parameter<int>("camera_center_offset", 7);

  this->declare_parameter<double>("preprocessing.resize_factor", 0.5);
  this->declare_parameter<int>("preprocessing.gray_threshold_low", 200);
  this->declare_parameter<int>("preprocessing.gray_threshold_high", 255);
  this->declare_parameter<int>("preprocessing.canny_low", 300);
  this->declare_parameter<int>("preprocessing.canny_high", 700);
  this->declare_parameter<int>("preprocessing.canny_aperture", 3);
  this->declare_parameter<int>("preprocessing.median_blur_kernel", 5);
  this->declare_parameter<int>("preprocessing.roi_left_x", 0);
  this->declare_parameter<int>("preprocessing.roi_left_y", 0);
  this->declare_parameter<int>("preprocessing.roi_left_width", 160);
  this->declare_parameter<int>("preprocessing.roi_left_height", 240);
  this->declare_parameter<int>("preprocessing.roi_right_x", 160);
  this->declare_parameter<int>("preprocessing.roi_right_y", 0);
  this->declare_parameter<int>("preprocessing.roi_right_width", 160);
  this->declare_parameter<int>("preprocessing.roi_right_height", 240);
  this->declare_parameter<bool>("preprocessing.apply_white_filter", true);
  this->declare_parameter<int>("preprocessing.white_filter_max_width", 20);
  this->declare_parameter<std::vector<double>>(
    "preprocessing.ipm_src_flat",
    {56.0, 110.0, 250.0, 110.0, 0.0, 200.0, 320.0, 200.0});
  this->declare_parameter<std::vector<double>>(
    "preprocessing.ipm_dst_flat",
    {56.0, 0.0, 250.0, 0.0, 56.0, 240.0, 250.0, 240.0});
}

void LaneDetectionSlidingWindowsNode::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  const auto t_start = std::chrono::steady_clock::now();

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // 1. Preprocesado IDÉNTICO al del FCM
  lane_common::PreprocessResult pp;
  try {
    pp = preprocessor_->process(cv_ptr->image);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Preprocessor failed: %s", e.what());
    return;
  }

  // 2. Sliding Windows sobre la imagen binaria
  const auto sw_result = sw_algorithm_->locate(pp.edges);

  // 3. Regresión polinomial cuadrática sobre cada lado
  const auto poly_left = fitQuadratic(
    sw_result.left.points, sw_params_.min_points_for_regression);
  const auto poly_right = fitQuadratic(
    sw_result.right.points, sw_params_.min_points_for_regression);

  // Guardar como último válido si lo es (sirve para fallback)
  if (poly_left.valid) {
    last_poly_left_ = poly_left;
  }
  if (poly_right.valid) {
    last_poly_right_ = poly_right;
  }

  // 4. Estimación de pose
  float angle_deg = 0.0f;
  float center_dev = 0.0f;
  uint8_t status = 0;
  cv::Mat overlay = pp.ipm_bgr.clone();

  // Dibujar las ventanas de búsqueda primero
  for (const auto & w : sw_result.left.windows) {
    cv::rectangle(overlay, w, cv::Scalar(0, 255, 255), 1);
  }
  for (const auto & w : sw_result.right.windows) {
    cv::rectangle(overlay, w, cv::Scalar(0, 255, 255), 1);
  }

  estimatePoseAndAngle(
    poly_left, poly_right,
    pp.ipm_bgr.cols, pp.ipm_bgr.rows,
    angle_deg, center_dev, status,
    overlay);

  const auto t_end = std::chrono::steady_clock::now();
  const double dt_ms =
    std::chrono::duration<double, std::milli>(t_end - t_start).count();

  // 5. Publicar topics
  {
    std_msgs::msg::Float32 m;
    m.data = center_dev;
    center_pub_->publish(m);
  }
  {
    std_msgs::msg::Float32 m;
    m.data = angle_deg;
    angle_pub_->publish(m);
  }
  {
    std_msgs::msg::Float32 m;
    m.data = static_cast<float>(dt_ms);
    time_pub_->publish(m);
  }
  {
    std_msgs::msg::UInt8 m;
    m.data = status;
    status_pub_->publish(m);
  }
  {
    std_msgs::msg::Float32 m;
    m.data = static_cast<float>(
      sw_result.left.points.size() + sw_result.right.points.size()) / 2.0f;
    points_pub_->publish(m);
  }

  if (publish_debug_image_ && debug_pub_) {
    std::stringstream ss;
    ss << "SW | angle=" << angle_deg << " dev=" << center_dev
       << " " << dt_ms << "ms | pts L=" << sw_result.left.points.size()
       << " R=" << sw_result.right.points.size();
    cv::putText(
      overlay, ss.str(), cv::Point(5, 15),
      cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = overlay;
    debug_pub_->publish(*out_msg.toImageMsg());
  }
}

void LaneDetectionSlidingWindowsNode::estimatePoseAndAngle(
  const PolynomialCoefficients & poly_left,
  const PolynomialCoefficients & poly_right,
  int image_cols,
  int image_rows,
  float & angle_deg,
  float & center_deviation_px,
  uint8_t & status_out,
  cv::Mat & debug_overlay)
{
  const int center_cam = (image_cols / 2) - camera_center_offset_;
  const int y_top = 0;
  const int y_mid = image_rows / 2;
  const int y_bot = image_rows - 1;

  const bool found_L = poly_left.valid;
  const bool found_R = poly_right.valid;

  int center_lane = center_cam;

  // Dibujar curvas detectadas paso a paso
  if (found_L) {
    for (int y = y_bot; y >= y_top; y -= 4) {
      const int x = static_cast<int>(evaluatePolynomial(poly_left, y));
      cv::circle(debug_overlay, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1);
    }
  }
  if (found_R) {
    for (int y = y_bot; y >= y_top; y -= 4) {
      const int x = static_cast<int>(evaluatePolynomial(poly_right, y));
      cv::circle(debug_overlay, cv::Point(x, y), 1, cv::Scalar(0, 255, 0), -1);
    }
  }

  if (found_L && found_R) {
    status_out = 3;
    const double xl_mid = evaluatePolynomial(poly_left, y_mid);
    const double xr_mid = evaluatePolynomial(poly_right, y_mid);
    center_lane = static_cast<int>((xl_mid + xr_mid) / 2.0);
  } else if (found_L) {
    status_out = 1;
    const double xl_mid = evaluatePolynomial(poly_left, y_mid);
    center_lane = static_cast<int>(xl_mid + 73.0);
  } else if (found_R) {
    status_out = 2;
    const double xr_mid = evaluatePolynomial(poly_right, y_mid);
    center_lane = static_cast<int>(xr_mid - 73.0);
  } else {
    status_out = 0;
  }

  center_deviation_px = static_cast<float>(center_cam - center_lane);

  const float dy = static_cast<float>(y_bot - y_mid);
  const float dx = static_cast<float>(center_cam - center_lane);
  if (std::fabs(dy) < 1e-6f) {
    angle_deg = 90.0f;
  } else {
    const float m = dx / dy;
    angle_deg = 90.0f - std::atan(m) * 180.0f / static_cast<float>(M_PI);
  }

  cv::circle(debug_overlay, cv::Point(center_lane, y_mid), 3, cv::Scalar(0, 255, 255), -1);
  cv::line(
    debug_overlay,
    cv::Point(center_cam, y_bot),
    cv::Point(center_lane, y_mid),
    cv::Scalar(0, 0, 255), 2);
}

}  // namespace lane_detection_sliding_windows
