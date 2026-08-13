// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_hough/LaneDetectionHoughNode.hpp"

#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <cmath>
#include <sstream>

namespace lane_detection_hough
{

LaneDetectionHoughNode::LaneDetectionHoughNode(
  const rclcpp::NodeOptions & options)
: Node("lane_detection_hough", options)
{
  declareAndLoadParameters();

  // Construir el preprocessor IDÉNTICO al de FCM/SW.
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
  hough_extractor_ = std::make_shared<HoughLaneExtractor>(hough_params_);

  const std::string input_topic = this->declare_parameter<std::string>(
    "input_image_topic", "/sensors/camera/color/image_rect_color");

  auto qos = rclcpp::SensorDataQoS();
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic, qos,
    std::bind(&LaneDetectionHoughNode::imageCallback, this, std::placeholders::_1));

  center_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/center_deviation", 10);
  angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/angle_deviation", 10);
  time_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/processing_time_ms", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("~/detection_status", 10);
  segments_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/segments_per_side", 10);

  if (publish_debug_image_) {
    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/debug_image", 1);
  }

  RCLCPP_INFO(
    this->get_logger(),
    "lane_detection_hough initialized. Subscribed to: %s "
    "| votes_thr=%d, min_len=%.1f, max_gap=%.1f, min_slope=%.2f",
    input_topic.c_str(),
    hough_params_.votes_threshold,
    hough_params_.min_line_length,
    hough_params_.max_line_gap,
    hough_params_.min_abs_slope);
}

void LaneDetectionHoughNode::declareAndLoadParameters()
{
  hough_params_.rho_resolution = this->declare_parameter<double>(
    "hough.rho_resolution", 1.0);
  hough_params_.theta_resolution = this->declare_parameter<double>(
    "hough.theta_resolution", CV_PI / 180.0);
  hough_params_.votes_threshold = this->declare_parameter<int>(
    "hough.votes_threshold", 25);
  hough_params_.min_line_length = this->declare_parameter<double>(
    "hough.min_line_length", 20.0);
  hough_params_.max_line_gap = this->declare_parameter<double>(
    "hough.max_line_gap", 10.0);
  hough_params_.min_abs_slope = this->declare_parameter<double>(
    "hough.min_abs_slope", 0.5);

  min_endpoints_for_regression_ = this->declare_parameter<int>(
    "hough.min_endpoints_for_regression", 4);

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

void LaneDetectionHoughNode::imageCallback(
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

  // 1. Preprocesado IDÉNTICO al de FCM y SW
  lane_common::PreprocessResult pp;
  try {
    pp = preprocessor_->process(cv_ptr->image);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Preprocessor failed: %s", e.what());
    return;
  }

  // 2. Extracción de segmentos Hough
  const int midline_x = pp.edges.cols / 2;
  const auto hough_result = hough_extractor_->extract(pp.edges, midline_x);

  // 3. Regresión lineal sobre los endpoints de cada lado
  const auto line_L = fitLineSimple(
    hough_result.left.endpoints, min_endpoints_for_regression_);
  const auto line_R = fitLineSimple(
    hough_result.right.endpoints, min_endpoints_for_regression_);

  // 4. Estimación de pose
  float angle_deg = 0.0f;
  float center_dev = 0.0f;
  uint8_t status = 0;
  cv::Mat overlay = pp.ipm_bgr.clone();

  // Dibujar los segmentos Hough crudos (debug)
  for (const auto & seg : hough_result.left.raw_segments) {
    cv::line(
      overlay,
      cv::Point(seg[0], seg[1]),
      cv::Point(seg[2], seg[3]),
      cv::Scalar(0, 200, 200), 1);
  }
  for (const auto & seg : hough_result.right.raw_segments) {
    cv::line(
      overlay,
      cv::Point(seg[0], seg[1]),
      cv::Point(seg[2], seg[3]),
      cv::Scalar(200, 0, 200), 1);
  }

  estimatePoseAndAngle(
    line_L, line_R,
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
      hough_result.left.raw_segments.size() +
      hough_result.right.raw_segments.size()) / 2.0f;
    segments_pub_->publish(m);
  }

  if (publish_debug_image_ && debug_pub_) {
    std::stringstream ss;
    ss << "HOUGH | angle=" << angle_deg << " dev=" << center_dev
       << " " << dt_ms << "ms | seg L=" << hough_result.left.raw_segments.size()
       << " R=" << hough_result.right.raw_segments.size();
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

void LaneDetectionHoughNode::estimatePoseAndAngle(
  const LineModel & line_L,
  const LineModel & line_R,
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

  const bool found_L = line_L.valid;
  const bool found_R = line_R.valid;

  int center_lane = center_cam;

  if (found_L) {
    cv::line(
      debug_overlay,
      cv::Point(static_cast<int>(xAtY(line_L, y_top)), y_top),
      cv::Point(static_cast<int>(xAtY(line_L, y_bot)), y_bot),
      cv::Scalar(0, 255, 0), 2);
  }
  if (found_R) {
    cv::line(
      debug_overlay,
      cv::Point(static_cast<int>(xAtY(line_R, y_top)), y_top),
      cv::Point(static_cast<int>(xAtY(line_R, y_bot)), y_bot),
      cv::Scalar(0, 255, 0), 2);
  }

  if (found_L && found_R) {
    status_out = 3;
    const float xl_mid = xAtY(line_L, y_mid);
    const float xr_mid = xAtY(line_R, y_mid);
    center_lane = static_cast<int>((xl_mid + xr_mid) / 2.0f);
  } else if (found_L) {
    status_out = 1;
    const float xl_mid = xAtY(line_L, y_mid);
    center_lane = static_cast<int>(xl_mid + 73.0f);
  } else if (found_R) {
    status_out = 2;
    const float xr_mid = xAtY(line_R, y_mid);
    center_lane = static_cast<int>(xr_mid - 73.0f);
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

}  // namespace lane_detection_hough
