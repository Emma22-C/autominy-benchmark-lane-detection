// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_sliding_windows/PolynomialRegression.hpp"

namespace lane_detection_sliding_windows
{

PolynomialCoefficients fitQuadratic(
  const std::vector<cv::Point> & points,
  int min_points)
{
  PolynomialCoefficients out;
  const int n = static_cast<int>(points.size());
  if (n < min_points) {
    return out;
  }

  // Ajustamos col = a0 + a1*row + a2*row^2 minimizando ||A*x - b||^2
  // donde cada fila de A es [1, row, row^2] y b es col.
  cv::Mat A(n, 3, CV_64F);
  cv::Mat b(n, 1, CV_64F);

  for (int i = 0; i < n; ++i) {
    const double row = static_cast<double>(points[i].y);  // row = y de la imagen
    const double col = static_cast<double>(points[i].x);  // col = x de la imagen
    A.at<double>(i, 0) = 1.0;
    A.at<double>(i, 1) = row;
    A.at<double>(i, 2) = row * row;
    b.at<double>(i, 0) = col;
  }

  cv::Mat x;
  const bool ok = cv::solve(A, b, x, cv::DECOMP_QR);
  if (!ok) {
    return out;
  }

  out.a0 = x.at<double>(0, 0);
  out.a1 = x.at<double>(1, 0);
  out.a2 = x.at<double>(2, 0);
  out.valid = true;
  return out;
}

}  // namespace lane_detection_sliding_windows
