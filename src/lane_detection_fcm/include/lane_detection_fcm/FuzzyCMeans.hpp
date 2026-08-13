// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_DETECTION_FCM__FUZZY_C_MEANS_HPP_
#define LANE_DETECTION_FCM__FUZZY_C_MEANS_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace lane_detection_fcm
{

/**
 * @brief Parámetros del algoritmo FCM.
 *
 * Valores por defecto idénticos a los reportados en la Tabla II
 * del paper original: m=2, epsilon=0.005, clusters=3 (aunque
 * lane_detectionfuzzy.cpp usaba N_CLUSTERS=4; ambos valores se
 * documentan y son configurables).
 */
struct FcmParams
{
  int n_clusters = 4;        // # centroides por ROI
  int n_dimensions = 2;      // (x, y)
  double m = 2.0;            // fuzziness exponent
  double epsilon = 0.005;    // criterio de paro
  int max_iterations = 50;   // safety cap (evita loops infinitos)
};

/**
 * @brief Resultado de una ejecución del FCM sobre un set de puntos.
 */
struct FcmResult
{
  /// Centroides finales, uno por cluster: c x d (típicamente N_CLUSTERS x 2).
  std::vector<std::vector<double>> centroids;
  /// Matriz de pertenencia: dataSize x N_CLUSTERS.
  std::vector<std::vector<double>> membership;
  /// Iteraciones que tomó converger.
  int iterations = 0;
  /// Convergió antes del max_iterations.
  bool converged = false;
  /// Valor final de la función objetivo J (Eq. 1 del paper).
  double objective = 0.0;
};

/**
 * @brief Implementación del algoritmo Fuzzy C-Means con seeding manual.
 *
 * Sigue el flujo del paper:
 *   1. Recibe centroides iniciales (seeding manual, no aleatorio).
 *   2. Itera: calcular U (membership) -> recalcular V (centroides).
 *   3. Para cuando max(|U^(j) - U^(j-1)|) < epsilon.
 *
 * Diferencias vs. el código ROS1 original:
 *   - El estado se resetea correctamente entre frames (no hay memory leak
 *     en los vectores tL/tR).
 *   - El cálculo de max_diff es consistente desde el frame 0
 *     (no depende del flag degreeL/degreeR no inicializado).
 *   - Función pura, fácil de testear y de instanciar para L y R.
 */
class FuzzyCMeans
{
public:
  FuzzyCMeans() = default;
  explicit FuzzyCMeans(const FcmParams & params);

  /**
   * @brief Ejecuta el FCM sobre un dataset 2D.
   * @param dataset Puntos a clasificar (en cv::Point en coords IPM).
   * @param initial_centroids Semilla manual (debe tener params.n_clusters filas).
   * @return Resultado con centroides finales, pertenencias y diagnóstico.
   *
   * Si el dataset está vacío o tiene menos puntos que clusters,
   * devuelve los centroides iniciales sin iterar.
   */
  FcmResult cluster(
    const std::vector<cv::Point> & dataset,
    const std::vector<std::vector<double>> & initial_centroids);

  /// Acceso a parámetros activos.
  const FcmParams & params() const {return params_;}

private:
  FcmParams params_;

  /// Distancia euclidiana al cluster j.
  double distance(
    const cv::Point & p,
    const std::vector<double> & centroid) const;

  /// Calcula un valor nuevo de u_ij según la Eq. 4.
  double newMembership(
    const cv::Point & point,
    int cluster_j,
    const std::vector<std::vector<double>> & centroids) const;

  /// Actualiza la matriz U y devuelve max_diff vs. la iteración previa.
  double updateMembership(
    const std::vector<cv::Point> & dataset,
    const std::vector<std::vector<double>> & centroids,
    std::vector<std::vector<double>> & membership,
    bool first_iteration);

  /// Actualiza los centroides según la Eq. 5.
  void updateCentroids(
    const std::vector<cv::Point> & dataset,
    const std::vector<std::vector<double>> & membership,
    std::vector<std::vector<double>> & centroids);

  /// Calcula el valor final de la función objetivo (Eq. 1).
  double computeObjective(
    const std::vector<cv::Point> & dataset,
    const std::vector<std::vector<double>> & centroids,
    const std::vector<std::vector<double>> & membership) const;
};

}  // namespace lane_detection_fcm

#endif  // LANE_DETECTION_FCM__FUZZY_C_MEANS_HPP_
