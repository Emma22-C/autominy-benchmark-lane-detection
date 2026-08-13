// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_DETECTION_FCM__LANE_DETECTION_FCM_NODE_HPP_
#define LANE_DETECTION_FCM__LANE_DETECTION_FCM_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "lane_common/Preprocessor.hpp"
#include "lane_detection_fcm/FuzzyCMeans.hpp"

#include <memory>
#include <vector>

namespace lane_detection_fcm
{

/**
 * @brief Nodo ROS2 que ejecuta FCM sobre la salida de lane_common::Preprocessor
 *        y publica el contrato estandarizado de detección de carril.
 *
 * Topics publicados (relativos al namespace del nodo):
 *   ~/center_deviation     std_msgs/Float32  (px, IPM space)
 *   ~/angle_deviation      std_msgs/Float32  (deg, vs vertical de la imagen)
 *   ~/processing_time_ms   std_msgs/Float32  (ms por frame)
 *   ~/detection_status     std_msgs/UInt8    (0=ninguno, 1=L, 2=R, 3=L+R)
 *   ~/debug_image          sensor_msgs/Image (BGR8, con overlay de centroides y líneas)
 *   ~/xie_beni_left        std_msgs/Float32  (Tabla I del paper)
 *   ~/xie_beni_right       std_msgs/Float32
 *   ~/fpc_left             std_msgs/Float32
 *   ~/fpc_right            std_msgs/Float32
 */
class LaneDetectionFcmNode : public rclcpp::Node
{
public:
  explicit LaneDetectionFcmNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callback principal
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  // Helpers
  void declareAndLoadParameters();
  std::vector<std::vector<double>> readSeedingFromParams(const std::string & side);

  // Estima ángulo y desviación a partir de los centroides L y R.
  // status_out se llena con: 0,1,2,3 según los lados detectados.
  void estimatePoseAndAngle(
    const std::vector<cv::Point> & centroids_left,
    const std::vector<cv::Point> & centroids_right,
    int image_cols,
    int image_rows,
    float & angle_deg,
    float & center_deviation_px,
    uint8_t & status_out,
    cv::Mat & debug_overlay);

  // ---- Atributos ----
  // Preprocesado compartido
  std::shared_ptr<lane_common::Preprocessor> preprocessor_;

  // Algoritmos FCM (uno por ROI, mantiene los centroides como estado caliente)
  std::shared_ptr<FuzzyCMeans> fcm_left_;
  std::shared_ptr<FuzzyCMeans> fcm_right_;

  // Centroides iniciales (semilla manual) — actualizados frame a frame con los últimos.
  std::vector<std::vector<double>> seed_left_;
  std::vector<std::vector<double>> seed_right_;

  // Parámetros del nodo
  FcmParams fcm_params_;
  bool publish_debug_image_;
  bool warm_start_seeding_;  // si true, usa centroides del frame anterior como semilla
  int camera_center_offset_;  // del paper: center_cam = (img.cols/2) - 7

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr center_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr xb_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr xb_right_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fpc_left_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fpc_right_pub_;
};

}  // namespace lane_detection_fcm

#endif  // LANE_DETECTION_FCM__LANE_DETECTION_FCM_NODE_HPP_
