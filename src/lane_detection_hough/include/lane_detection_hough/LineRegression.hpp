// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_DETECTION_HOUGH__LINE_REGRESSION_HPP_
#define LANE_DETECTION_HOUGH__LINE_REGRESSION_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace lane_detection_hough
{

/**
 * @brief Parámetros de la línea ajustada en formato cv::fitLine.
 *
 * Una línea de fitLine se representa como (vx, vy, x0, y0) donde
 * (vx, vy) es el vector dirección unitario y (x0, y0) un punto en la
 * línea. Para un Y dado: x = x0 + (vx/vy) * (Y - y0).
 *
 * El campo `valid` indica si había suficientes puntos para el ajuste
 * (mínimo 2 para una recta).
 */
struct LineModel
{
  float vx = 0.0f;
  float vy = 1.0f;
  float x0 = 0.0f;
  float y0 = 0.0f;
  bool valid = false;
};

/**
 * @brief Ajusta una recta a un conjunto de puntos usando cv::fitLine
 *        con norma L2 (mínimos cuadrados, robusto a outliers leves).
 *
 * Idéntica formulación a la usada en el nodo FCM sobre sus centroides:
 * esto garantiza que la única diferencia entre FCM y Hough sea
 * **qué puntos se le pasan al ajuste**, no cómo se ajustan.
 *
 * @param points Endpoints de segmentos Hough.
 * @param min_points Mínimo requerido para considerar válido el ajuste.
 * @return Modelo de línea; valid=false si insuficientes puntos.
 */
LineModel fitLineSimple(
  const std::vector<cv::Point> & points,
  int min_points = 4);

/**
 * @brief Evalúa la línea a un Y dado: devuelve X correspondiente.
 *
 * Maneja el caso |vy| ~ 0 (línea casi horizontal) devolviendo x0.
 */
inline float xAtY(const LineModel & m, float y)
{
  if (std::fabs(m.vy) < 1e-6f) {
    return m.x0;
  }
  return m.x0 + (m.vx / m.vy) * (y - m.y0);
}

}  // namespace lane_detection_hough

#endif  // LANE_DETECTION_HOUGH__LINE_REGRESSION_HPP_
