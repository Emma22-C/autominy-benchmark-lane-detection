// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_fcm/LaneDetectionFcmNode.hpp"
#include "lane_detection_fcm/ClusterMetrics.hpp"
#include "lane_common/PointExtractor.hpp"

#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <sstream>

namespace lane_detection_fcm
{

LaneDetectionFcmNode::LaneDetectionFcmNode(const rclcpp::NodeOptions & options)
: Node("lane_detection_fcm", options)
{
  declareAndLoadParameters();

  // Construir el preprocessor con los parámetros declarados en lane_common
  lane_common::PreprocessorParams pp_params;
  // (Los YAML se cargan desde el launch; aquí solo si vienen en el namespace.)
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

  // IPM points (cargados desde el YAML como vectores planos de 8 doubles
  // cada uno: [x0,y0,x1,y1,x2,y2,x3,y3]). Si no están en el YAML, se
  // mantienen los defaults del PreprocessorParams struct.
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

  // Construir los FCM
  fcm_left_ = std::make_shared<FuzzyCMeans>(fcm_params_);
  fcm_right_ = std::make_shared<FuzzyCMeans>(fcm_params_);

  // Cargar seeding inicial
  seed_left_ = readSeedingFromParams("left");
  seed_right_ = readSeedingFromParams("right");

  // QoS y suscriptor
  const std::string input_topic = this->declare_parameter<std::string>(
    "input_image_topic", "/sensors/camera/color/image_rect_color");

  auto qos = rclcpp::SensorDataQoS();
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic, qos,
    std::bind(&LaneDetectionFcmNode::imageCallback, this, std::placeholders::_1));

  // Publishers (todos relativos al namespace del nodo)
  center_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/center_deviation", 10);
  angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/angle_deviation", 10);
  time_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/processing_time_ms", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::UInt8>("~/detection_status", 10);

  if (publish_debug_image_) {
    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("~/debug_image", 1);
  }

  xb_left_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/xie_beni_left", 10);
  xb_right_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/xie_beni_right", 10);
  fpc_left_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/fpc_left", 10);
  fpc_right_pub_ = this->create_publisher<std_msgs::msg::Float32>("~/fpc_right", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "lane_detection_fcm initialized. Subscribed to: %s | clusters=%d, m=%.2f, epsilon=%.4f",
    input_topic.c_str(),
    fcm_params_.n_clusters, fcm_params_.m, fcm_params_.epsilon);
}

void LaneDetectionFcmNode::declareAndLoadParameters()
{
  fcm_params_.n_clusters = this->declare_parameter<int>("fcm.n_clusters", 4);
  fcm_params_.n_dimensions = 2;
  fcm_params_.m = this->declare_parameter<double>("fcm.m", 2.0);
  fcm_params_.epsilon = this->declare_parameter<double>("fcm.epsilon", 0.005);
  fcm_params_.max_iterations = this->declare_parameter<int>("fcm.max_iterations", 50);

  publish_debug_image_ = this->declare_parameter<bool>("publish_debug_image", true);
  warm_start_seeding_ = this->declare_parameter<bool>("warm_start_seeding", false);
  camera_center_offset_ = this->declare_parameter<int>("camera_center_offset", 7);

  // Preprocesado: los declaramos para que sean configurables desde el mismo YAML.
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
  // IPM como vector plano de 8 doubles cada uno (x0,y0,x1,y1,...).
  this->declare_parameter<std::vector<double>>(
    "preprocessing.ipm_src_flat",
    {56.0, 110.0, 250.0, 110.0, 0.0, 200.0, 320.0, 200.0});
  this->declare_parameter<std::vector<double>>(
    "preprocessing.ipm_dst_flat",
    {56.0, 0.0, 250.0, 0.0, 56.0, 240.0, 250.0, 240.0});
}

std::vector<std::vector<double>> LaneDetectionFcmNode::readSeedingFromParams(
  const std::string & side)
{
  // Seeding por defecto idéntico al paper original (4 centroides equiespaciados
  // a lo largo del eje Y, en X fijo: 80 para L, 240 para R).
  std::vector<double> default_x_coords;
  std::vector<double> default_y_coords;

  if (side == "left") {
    default_x_coords = {80.0, 80.0, 80.0, 80.0};
    default_y_coords = {48.0, 96.0, 144.0, 192.0};
  } else {
    default_x_coords = {240.0, 240.0, 240.0, 240.0};
    default_y_coords = {48.0, 96.0, 144.0, 192.0};
  }

  // Truncar/expandir según n_clusters
  default_x_coords.resize(fcm_params_.n_clusters, default_x_coords.back());
  default_y_coords.resize(fcm_params_.n_clusters, default_y_coords.back());

  auto xs = this->declare_parameter<std::vector<double>>(
    "fcm.seed_" + side + "_x", default_x_coords);
  auto ys = this->declare_parameter<std::vector<double>>(
    "fcm.seed_" + side + "_y", default_y_coords);

  if (static_cast<int>(xs.size()) != fcm_params_.n_clusters ||
    static_cast<int>(ys.size()) != fcm_params_.n_clusters)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Seed size for side=%s mismatches n_clusters=%d; using defaults.",
      side.c_str(), fcm_params_.n_clusters);
    xs = default_x_coords;
    ys = default_y_coords;
  }

  std::vector<std::vector<double>> seed;
  seed.reserve(fcm_params_.n_clusters);
  for (int i = 0; i < fcm_params_.n_clusters; ++i) {
    seed.push_back({xs[i], ys[i]});
  }
  return seed;
}

void LaneDetectionFcmNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  const auto t_start = std::chrono::steady_clock::now();

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // 1. Preprocesado compartido
  lane_common::PreprocessResult pp;
  try {
    pp = preprocessor_->process(cv_ptr->image);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Preprocessor failed: %s", e.what());
    return;
  }

  // 2. Extraer puntos por ROI
  auto pts = lane_common::extractPoints(
    pp.edges,
    preprocessor_->getLeftRoi(),
    preprocessor_->getRightRoi());

  // 3. FCM por ROI
  FcmResult res_left = fcm_left_->cluster(pts.left, seed_left_);
  FcmResult res_right = fcm_right_->cluster(pts.right, seed_right_);

  // 4. Métricas internas (Tabla I)
  const double xb_left = xieBeniIndex(pts.left, res_left, fcm_params_.m);
  const double xb_right = xieBeniIndex(pts.right, res_right, fcm_params_.m);
  const double fpc_left = fuzzyPartitionCoefficient(res_left);
  const double fpc_right = fuzzyPartitionCoefficient(res_right);

  // 5. Convertir centroides a cv::Point ordenados por Y (replica el paper)
  auto centroids_to_points = [](const std::vector<std::vector<double>> & cs) {
      std::vector<cv::Point> out;
      out.reserve(cs.size());
      for (const auto & c : cs) {
        out.emplace_back(
          static_cast<int>(std::round(c[0])),
          static_cast<int>(std::round(c[1])));
      }
      std::sort(
        out.begin(), out.end(),
        [](const cv::Point & a, const cv::Point & b) {return a.y < b.y;});
      return out;
    };
  const auto centroids_L = centroids_to_points(res_left.centroids);
  const auto centroids_R = centroids_to_points(res_right.centroids);

  // 6. Estimar ángulo y desviación (replica estimate_posAng del paper)
  float angle_deg = 0.0f;
  float center_dev = 0.0f;
  uint8_t status = 0;
  cv::Mat overlay = pp.ipm_bgr.clone();

  estimatePoseAndAngle(
    centroids_L, centroids_R,
    pp.ipm_bgr.cols, pp.ipm_bgr.rows,
    angle_deg, center_dev, status,
    overlay);

  // 7. Warm-start seeding opcional (próximo frame parte de los centroides actuales)
  if (warm_start_seeding_ && status != 0) {
    seed_left_ = res_left.centroids;
    seed_right_ = res_right.centroids;
  }

  const auto t_end = std::chrono::steady_clock::now();
  const double dt_ms =
    std::chrono::duration<double, std::milli>(t_end - t_start).count();

  // 8. Publicar topics
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
    m.data = std::isfinite(xb_left) ? static_cast<float>(xb_left) : -1.0f;
    xb_left_pub_->publish(m);
  }
  {
    std_msgs::msg::Float32 m;
    m.data = std::isfinite(xb_right) ? static_cast<float>(xb_right) : -1.0f;
    xb_right_pub_->publish(m);
  }
  {
    std_msgs::msg::Float32 m;
    m.data = std::isfinite(fpc_left) ? static_cast<float>(fpc_left) : -1.0f;
    fpc_left_pub_->publish(m);
  }
  {
    std_msgs::msg::Float32 m;
    m.data = std::isfinite(fpc_right) ? static_cast<float>(fpc_right) : -1.0f;
    fpc_right_pub_->publish(m);
  }

  if (publish_debug_image_ && debug_pub_) {
    // Dibujar centroides finales
    for (size_t i = 0; i < centroids_L.size(); ++i) {
      cv::circle(overlay, centroids_L[i], 4, cv::Scalar(0, 255, 255), -1);
    }
    for (size_t i = 0; i < centroids_R.size(); ++i) {
      cv::circle(overlay, centroids_R[i], 4, cv::Scalar(255, 0, 255), -1);
    }
    // Texto informativo
    std::stringstream ss;
    ss << "FCM | angle=" << angle_deg << " dev=" << center_dev
       << " " << dt_ms << "ms";
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

void LaneDetectionFcmNode::estimatePoseAndAngle(
  const std::vector<cv::Point> & centroids_left,
  const std::vector<cv::Point> & centroids_right,
  int image_cols,
  int image_rows,
  float & angle_deg,
  float & center_deviation_px,
  uint8_t & status_out,
  cv::Mat & debug_overlay)
{
  // Replica del estimate_posAng() del paper, usando cv::fitLine para ajustar
  // una recta a cada conjunto de centroides (regresión lineal robusta).

  const int center_cam = (image_cols / 2) - camera_center_offset_;
  const int y_top = 0;
  const int y_mid = image_rows / 2;        // ~120 si img 320x240
  const int y_bot = image_rows;             // 240

  bool found_left = false;
  bool found_right = false;
  cv::Vec4f line_L, line_R;

  // Necesitamos al menos 2 centroides para ajustar una recta.
  if (centroids_left.size() >= 2) {
    cv::fitLine(centroids_left, line_L, cv::DIST_L2, 0, 0.01, 0.01);
    found_left = true;
  }
  if (centroids_right.size() >= 2) {
    cv::fitLine(centroids_right, line_R, cv::DIST_L2, 0, 0.01, 0.01);
    found_right = true;
  }

  // line_X = (vx, vy, x0, y0). La recta pasa por (x0,y0) con dirección (vx,vy).
  // Para un Y dado: x = x0 + (vx/vy) * (Y - y0).
  auto x_at_y = [](const cv::Vec4f & l, int y) {
      const float vx = l[0];
      const float vy = l[1];
      const float x0 = l[2];
      const float y0 = l[3];
      if (std::fabs(vy) < 1e-6f) {
        // Línea horizontal; usamos x0.
        return x0;
      }
      return x0 + (vx / vy) * (static_cast<float>(y) - y0);
    };

  int center_lane = center_cam;  // si no hay nada, asumimos sin desvío

  if (found_left && found_right) {
    status_out = 3;
    const float xl_mid = x_at_y(line_L, y_mid);
    const float xr_mid = x_at_y(line_R, y_mid);
    center_lane = static_cast<int>((xl_mid + xr_mid) / 2.0f);

    cv::line(
      debug_overlay,
      cv::Point(static_cast<int>(x_at_y(line_L, y_top)), y_top),
      cv::Point(static_cast<int>(x_at_y(line_L, y_bot)), y_bot),
      cv::Scalar(0, 255, 0), 2);
    cv::line(
      debug_overlay,
      cv::Point(static_cast<int>(x_at_y(line_R, y_top)), y_top),
      cv::Point(static_cast<int>(x_at_y(line_R, y_bot)), y_bot),
      cv::Scalar(0, 255, 0), 2);
  } else if (found_left) {
    status_out = 1;
    // Asumimos un offset fijo entre línea L y centro del carril (~73 px en el paper)
    const float xl_mid = x_at_y(line_L, y_mid);
    center_lane = static_cast<int>(xl_mid + 73.0f);
    cv::line(
      debug_overlay,
      cv::Point(static_cast<int>(x_at_y(line_L, y_top)), y_top),
      cv::Point(static_cast<int>(x_at_y(line_L, y_bot)), y_bot),
      cv::Scalar(0, 255, 0), 2);
  } else if (found_right) {
    status_out = 2;
    const float xr_mid = x_at_y(line_R, y_mid);
    center_lane = static_cast<int>(xr_mid - 73.0f);
    cv::line(
      debug_overlay,
      cv::Point(static_cast<int>(x_at_y(line_R, y_top)), y_top),
      cv::Point(static_cast<int>(x_at_y(line_R, y_bot)), y_bot),
      cv::Scalar(0, 255, 0), 2);
  } else {
    status_out = 0;
  }

  // Desviación
  center_deviation_px = static_cast<float>(center_cam - center_lane);

  // Ángulo (Eq. 10 y 11 del paper)
  const float dy = static_cast<float>(y_bot - y_mid);
  const float dx = static_cast<float>(center_cam - center_lane);
  if (std::fabs(dy) < 1e-6f) {
    angle_deg = 90.0f;
  } else {
    const float m = dx / dy;
    angle_deg = 90.0f - std::atan(m) * 180.0f / static_cast<float>(M_PI);
  }

  // Cámara y centro del carril en la imagen
  cv::circle(debug_overlay, cv::Point(center_lane, y_mid), 3, cv::Scalar(0, 255, 255), -1);
  cv::line(
    debug_overlay,
    cv::Point(center_cam, y_bot),
    cv::Point(center_lane, y_mid),
    cv::Scalar(0, 0, 255), 2);
}

}  // namespace lane_detection_fcm
