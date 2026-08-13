// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_COMMON__POINT_EXTRACTOR_HPP_
#define LANE_COMMON__POINT_EXTRACTOR_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace lane_common
{

/**
 * @brief Resultado de la extracción de puntos blancos por ROI.
 *
 * Los puntos están en coordenadas absolutas de la imagen IPM,
 * no relativas al ROI. Esto facilita la regresión y el overlay.
 */
struct ExtractedPoints
{
  /// Puntos blancos (>0) dentro del ROI izquierdo, formato (x,y) en cv::Point.
  std::vector<cv::Point> left;
  /// Puntos blancos dentro del ROI derecho.
  std::vector<cv::Point> right;
};

/**
 * @brief Extrae los píxeles blancos de la imagen binaria (Canny)
 *        dentro de cada ROI, idéntico para FCM y Sliding Windows.
 *
 * Replica el comportamiento de GetPoints() del paper original, pero
 * separando explícitamente los puntos por ROI y dejando la decisión
 * de qué hacer con ellos al algoritmo (clustering o ventanas).
 *
 * @param edges Imagen binaria 8UC1 (salida del Canny del Preprocessor).
 * @param roi_left Rectángulo del ROI izquierdo.
 * @param roi_right Rectángulo del ROI derecho.
 * @param edge_margin Margen de borde a ignorar (replica el ">=2" del original).
 * @return Puntos blancos por ROI.
 */
ExtractedPoints extractPoints(
  const cv::Mat & edges,
  const cv::Rect & roi_left,
  const cv::Rect & roi_right,
  int edge_margin = 2);

}  // namespace lane_common

#endif  // LANE_COMMON__POINT_EXTRACTOR_HPP_
