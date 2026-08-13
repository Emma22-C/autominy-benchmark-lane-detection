// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_fcm/ClusterMetrics.hpp"
#include <cmath>
#include <limits>

namespace lane_detection_fcm
{

double xieBeniIndex(
  const std::vector<cv::Point> & dataset,
  const FcmResult & result,
  double m)
{
  const int n = static_cast<int>(dataset.size());
  const int c = static_cast<int>(result.centroids.size());
  if (n == 0 || c < 2) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (static_cast<int>(result.membership.size()) != n) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Numerador: sum_i sum_j u_ij^m * ||x_i - v_j||^2
  double numerator = 0.0;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < c; ++j) {
      const double dx = static_cast<double>(dataset[i].x) - result.centroids[j][0];
      const double dy = static_cast<double>(dataset[i].y) - result.centroids[j][1];
      const double dist2 = dx * dx + dy * dy;
      numerator += std::pow(result.membership[i][j], m) * dist2;
    }
  }

  // Denominador: N * min_{j != k} ||v_j - v_k||^2
  double min_centroid_dist2 = std::numeric_limits<double>::max();
  for (int j = 0; j < c; ++j) {
    for (int k = j + 1; k < c; ++k) {
      const double dx = result.centroids[j][0] - result.centroids[k][0];
      const double dy = result.centroids[j][1] - result.centroids[k][1];
      const double d2 = dx * dx + dy * dy;
      if (d2 < min_centroid_dist2) {
        min_centroid_dist2 = d2;
      }
    }
  }

  if (min_centroid_dist2 < 1e-12) {
    // Centroides colapsados; XB no es definible.
    return std::numeric_limits<double>::infinity();
  }

  return numerator / (static_cast<double>(n) * min_centroid_dist2);
}

double fuzzyPartitionCoefficient(const FcmResult & result)
{
  const int n = static_cast<int>(result.membership.size());
  if (n == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const int c = result.membership[0].empty() ?
    0 :
    static_cast<int>(result.membership[0].size());
  if (c == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < c; ++j) {
      sum += result.membership[i][j] * result.membership[i][j];
    }
  }
  return sum / static_cast<double>(n);
}

}  // namespace lane_detection_fcm
