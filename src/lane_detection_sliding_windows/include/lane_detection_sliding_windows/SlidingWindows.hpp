// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_DETECTION_SLIDING_WINDOWS__SLIDING_WINDOWS_HPP_
#define LANE_DETECTION_SLIDING_WINDOWS__SLIDING_WINDOWS_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace lane_detection_sliding_windows
{

/**
 * @brief Parámetros del algoritmo de Sliding Windows.
 *
 * Replica los valores hardcoded del paper original
 * (lane_detectionslide.cpp): n_windows=9, margin=20, minpix=40.
 * El threshold de validez de la regresión es 75 puntos.
 */
struct SlidingWindowsParams
{
  int n_windows = 9;
  int margin = 20;
  int min_pixels_per_window = 40;
  int min_points_for_regression = 75;
};

/**
 * @brief Resultado de la búsqueda de ventanas para un lado.
 *
 * Cada ventana es un cv::Rect (para overlay/debug) y la lista
 * `points` contiene los píxeles blancos encontrados dentro del
 * conjunto de ventanas (a usar como entrada para la regresión).
 */
struct SlidingWindowsLineResult
{
  /// Rectángulos de cada ventana, en coordenadas IPM.
  std::vector<cv::Rect> windows;
  /// Píxeles blancos encontrados, en coordenadas IPM.
  std::vector<cv::Point> points;
  /// Píxel x de arranque (max del histograma para este lado).
  int starting_x = 0;
};

/**
 * @brief Resultado conjunto: L y R.
 */
struct SlidingWindowsResult
{
  SlidingWindowsLineResult left;
  SlidingWindowsLineResult right;
};

/**
 * @brief Implementación del Sliding Windows con histograma como inicialización.
 *
 * El flujo replica el del paper original:
 *   1. Histogram() de la mitad inferior de la imagen IPM para encontrar
 *      los picos izquierdo y derecho de la franja blanca.
 *   2. locating_lines(): construir n_windows verticales centradas en cada
 *      pico; los píxeles blancos dentro de cada ventana se agregan al
 *      conjunto de puntos del lado correspondiente. Si una ventana tiene
 *      >= min_pixels_per_window, el centro de la siguiente ventana se
 *      reajusta al promedio en X de los píxeles encontrados.
 *
 * Diferencias con el código original:
 *   - No depende del Canny propio: recibe la imagen binaria preprocesada
 *     por lane_common::Preprocessor, exactamente igual que el FCM.
 *   - Los puntos se devuelven sin ser drenados/mutados internamente.
 */
class SlidingWindows
{
public:
  SlidingWindows() = default;
  explicit SlidingWindows(const SlidingWindowsParams & params);

  /**
   * @brief Ejecuta el pipeline sobre la imagen binaria.
   *
   * @param edges Imagen 8UC1 (salida del Canny del Preprocessor).
   * @return Ventanas y puntos por lado.
   */
  SlidingWindowsResult locate(const cv::Mat & edges);

  const SlidingWindowsParams & params() const {return params_;}

private:
  SlidingWindowsParams params_;

  /// Encuentra los dos picos del histograma vertical de la franja inferior.
  /// Devuelve [left_x, right_x].
  std::pair<int, int> histogramPeaks(const cv::Mat & edges) const;
};

}  // namespace lane_detection_sliding_windows

#endif  // LANE_DETECTION_SLIDING_WINDOWS__SLIDING_WINDOWS_HPP_
