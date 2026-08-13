// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_fcm/FuzzyCMeans.hpp"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace lane_detection_fcm
{

FuzzyCMeans::FuzzyCMeans(const FcmParams & params)
: params_(params)
{
}

double FuzzyCMeans::distance(
  const cv::Point & p,
  const std::vector<double> & centroid) const
{
  // Euclidiana 2D
  const double dx = static_cast<double>(p.x) - centroid[0];
  const double dy = static_cast<double>(p.y) - centroid[1];
  return std::sqrt(dx * dx + dy * dy);
}

double FuzzyCMeans::newMembership(
  const cv::Point & point,
  int cluster_j,
  const std::vector<std::vector<double>> & centroids) const
{
  // Eq. 4 del paper:
  //   u_ij = 1 / sum_k ( d_ij / d_ik )^(2/(m-1))
  const double exponent = 2.0 / (params_.m - 1.0);
  const double d_ij = distance(point, centroids[cluster_j]);

  // Caso degenerado: punto coincide con un centroide -> u_ij = 1, otros = 0.
  // Aquí lo manejamos asignando un epsilon mínimo para no dividir entre 0.
  constexpr double kMinDist = 1e-10;

  double sum = 0.0;
  for (int k = 0; k < params_.n_clusters; ++k) {
    const double d_ik = std::max(distance(point, centroids[k]), kMinDist);
    const double ratio = std::max(d_ij, kMinDist) / d_ik;
    sum += std::pow(ratio, exponent);
  }
  if (sum < kMinDist) {
    return 1.0 / params_.n_clusters;
  }
  return 1.0 / sum;
}

double FuzzyCMeans::updateMembership(
  const std::vector<cv::Point> & dataset,
  const std::vector<std::vector<double>> & centroids,
  std::vector<std::vector<double>> & membership,
  bool first_iteration)
{
  double max_diff = 0.0;
  const int n = static_cast<int>(dataset.size());

  for (int j = 0; j < params_.n_clusters; ++j) {
    for (int i = 0; i < n; ++i) {
      const double new_u = newMembership(dataset[i], j, centroids);
      const double old_u = membership[i][j];
      // En la primera iteración membership está en 0, así que max_diff
      // se calcula sobre new_u directamente (replica el comportamiento
      // del paper en el primer frame, pero de forma consistente).
      const double diff = first_iteration ?
        new_u :
        std::fabs(new_u - old_u);
      if (diff > max_diff) {
        max_diff = diff;
      }
      membership[i][j] = new_u;
    }
  }
  return max_diff;
}

void FuzzyCMeans::updateCentroids(
  const std::vector<cv::Point> & dataset,
  const std::vector<std::vector<double>> & membership,
  std::vector<std::vector<double>> & centroids)
{
  // Eq. 5 del paper:
  //   c_j = sum_i ( u_ij^m * z_i ) / sum_i ( u_ij^m )
  const int n = static_cast<int>(dataset.size());

  for (int j = 0; j < params_.n_clusters; ++j) {
    double denom = 0.0;
    double num_x = 0.0;
    double num_y = 0.0;
    for (int i = 0; i < n; ++i) {
      const double w = std::pow(membership[i][j], params_.m);
      denom += w;
      num_x += w * static_cast<double>(dataset[i].x);
      num_y += w * static_cast<double>(dataset[i].y);
    }
    if (denom > 1e-12) {
      centroids[j][0] = num_x / denom;
      centroids[j][1] = num_y / denom;
    }
    // Si denom es 0, mantenemos el centroide previo (no debería ocurrir
    // con un dataset no vacío, pero blindamos por si acaso).
  }
}

double FuzzyCMeans::computeObjective(
  const std::vector<cv::Point> & dataset,
  const std::vector<std::vector<double>> & centroids,
  const std::vector<std::vector<double>> & membership) const
{
  // Eq. 1 del paper:
  //   J = sum_i sum_j ( u_ij^m * d_ij^2 )
  double J = 0.0;
  const int n = static_cast<int>(dataset.size());
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < params_.n_clusters; ++j) {
      const double d = distance(dataset[i], centroids[j]);
      J += std::pow(membership[i][j], params_.m) * d * d;
    }
  }
  return J;
}

FcmResult FuzzyCMeans::cluster(
  const std::vector<cv::Point> & dataset,
  const std::vector<std::vector<double>> & initial_centroids)
{
  FcmResult result;
  result.centroids = initial_centroids;

  // Validaciones tempranas
  if (static_cast<int>(initial_centroids.size()) != params_.n_clusters) {
    throw std::invalid_argument(
            "FuzzyCMeans::cluster: initial_centroids size != n_clusters");
  }
  const int n = static_cast<int>(dataset.size());
  if (n < params_.n_clusters) {
    // No hay suficientes puntos para clustering significativo;
    // devolvemos los centroides iniciales tal cual.
    result.iterations = 0;
    result.converged = false;
    return result;
  }

  // Inicializar matriz de pertenencia con ceros.
  result.membership.assign(
    n, std::vector<double>(params_.n_clusters, 0.0));

  // Loop principal del FCM
  bool first = true;
  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    const double max_diff = updateMembership(
      dataset, result.centroids, result.membership, first);
    updateCentroids(dataset, result.membership, result.centroids);

    result.iterations = iter + 1;

    if (!first && max_diff < params_.epsilon) {
      result.converged = true;
      break;
    }
    first = false;
  }

  result.objective = computeObjective(
    dataset, result.centroids, result.membership);

  return result;
}

}  // namespace lane_detection_fcm
