// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_DETECTION_FCM__CLUSTER_METRICS_HPP_
#define LANE_DETECTION_FCM__CLUSTER_METRICS_HPP_

#include "lane_detection_fcm/FuzzyCMeans.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

namespace lane_detection_fcm
{

/**
 * @brief Índice Xie-Beni para validez del clustering fuzzy.
 *
 *   XB = ( sum_i sum_j u_ij^m * ||x_i - v_j||^2 ) / ( N * min_{j!=k} ||v_j - v_k||^2 )
 *
 * Menor es mejor: indica clusters compactos y bien separados.
 * Reportado en la Tabla I del paper original.
 *
 * @return Valor del índice, o NaN si no se puede calcular
 *         (p.ej. <2 clusters o dataset vacío).
 */
double xieBeniIndex(
  const std::vector<cv::Point> & dataset,
  const FcmResult & result,
  double m);

/**
 * @brief Fuzzy Partition Coefficient (FPC).
 *
 *   FPC = (1/N) * sum_i sum_j u_ij^2
 *
 * Valor en [1/c, 1]; mayor es mejor (1 = partición dura, 1/c = totalmente difusa).
 * Reportado en la Tabla I del paper original.
 */
double fuzzyPartitionCoefficient(const FcmResult & result);

}  // namespace lane_detection_fcm

#endif  // LANE_DETECTION_FCM__CLUSTER_METRICS_HPP_
