// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_DETECTION_SLIDING_WINDOWS__POLYNOMIAL_REGRESSION_HPP_
#define LANE_DETECTION_SLIDING_WINDOWS__POLYNOMIAL_REGRESSION_HPP_

#include <opencv2/opencv.hpp>
#include <vector>
#include <array>

namespace lane_detection_sliding_windows
{

/**
 * @brief Coeficientes de un polinomio cuadrático y = a0 + a1*x + a2*x^2.
 *
 * IMPORTANTE: Siguiendo el paper original, la regresión se hace de
 * y en función de x, donde x = row (y de la imagen) y y = column.
 * O sea, x_imagen = a0 + a1 * y_imagen + a2 * y_imagen^2.
 * Esto refleja que las líneas del carril son aproximadamente verticales.
 */
struct PolynomialCoefficients
{
  double a0 = 0.0;
  double a1 = 0.0;
  double a2 = 0.0;
  bool valid = false;
};

/**
 * @brief Ajusta un polinomio cuadrático a un conjunto de puntos.
 *
 * Usa OpenCV cv::solve con DECOMP_QR (más estable que la eliminación
 * Gaussiana manual del código original).
 *
 * Para que sea consistente con el paper, se ajusta:
 *   col = a0 + a1 * row + a2 * row^2
 *
 * O sea, los "x" del polinomio son las coordenadas Y de los puntos en
 * la imagen, y los "y" son las coordenadas X. Esto explota que las
 * líneas del carril son casi verticales y evita problemas de
 * pendientes infinitas.
 *
 * @param points Puntos en formato cv::Point(x_col, y_row).
 * @param min_points Mínimo requerido para considerar válido el ajuste.
 * @return Coeficientes; valid=false si no había suficientes puntos.
 */
PolynomialCoefficients fitQuadratic(
  const std::vector<cv::Point> & points,
  int min_points = 75);

/// Evalúa el polinomio: col(row) = a0 + a1*row + a2*row^2.
inline double evaluatePolynomial(const PolynomialCoefficients & p, double row)
{
  return p.a0 + p.a1 * row + p.a2 * row * row;
}

}  // namespace lane_detection_sliding_windows

#endif  // LANE_DETECTION_SLIDING_WINDOWS__POLYNOMIAL_REGRESSION_HPP_
