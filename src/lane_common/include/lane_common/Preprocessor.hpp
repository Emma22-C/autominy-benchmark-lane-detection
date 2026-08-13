// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_COMMON__PREPROCESSOR_HPP_
#define LANE_COMMON__PREPROCESSOR_HPP_

#include <opencv2/opencv.hpp>
#include <array>
#include <string>

namespace lane_common
{

/**
 * @brief Parámetros de preprocesado, configurables vía YAML.
 *
 * Por defecto se usan los mismos valores que el paper original
 * (Vidal-Cuevas et al., AutoMiny v4.0, Berlin Virtual Lab):
 * - Resize a 320x240 (factor 0.5 sobre 640x480)
 * - 4 puntos fuente del IPM en coordenadas de la imagen redimensionada
 * - Canny con umbrales (300, 700)
 *
 * Para que la comparación FCM vs Sliding Windows sea justa,
 * AMBOS algoritmos deben usar la misma instancia de Preprocessor.
 */
struct PreprocessorParams
{
  // Resize
  double resize_factor = 0.5;       // 640x480 -> 320x240

  // IPM (4 puntos fuente, 4 puntos destino) en imagen redimensionada
  // Mismos puntos que lane_detectionfuzzy.cpp - source_points1[]
  std::array<cv::Point2f, 4> ipm_src {
    cv::Point2f(56, 110),
    cv::Point2f(250, 110),
    cv::Point2f(0, 200),
    cv::Point2f(320, 200)
  };
  std::array<cv::Point2f, 4> ipm_dst {
    cv::Point2f(56, 0),
    cv::Point2f(250, 0),
    cv::Point2f(56, 240),
    cv::Point2f(250, 240)
  };

  // Threshold + Canny
  int gray_threshold_low = 200;     // inRange
  int gray_threshold_high = 255;
  int canny_low = 300;
  int canny_high = 700;
  int canny_aperture = 3;
  int median_blur_kernel = 5;

  // White Filter (suppression of overly-wide horizontal white runs).
  // Replicates white_filter() from lane_detectionslide.cpp: scans each
  // row of the binary image and zeroes out any continuous white run
  // wider than `white_filter_max_width` pixels. This was originally
  // introduced to remove crosswalk-like patterns and other wide blobs
  // that don't look like a lane stripe. Lane stripes are thin (4-8 px
  // in IPM space) so they pass through; crosswalks (20-40 px wide
  // blocks) get removed.
  //
  // Crucial for the TMR track which contains crosswalk-style
  // intersections in the middle. Disabled per default would bias the
  // FCM-vs-SW comparison (since the original SW code had this filter
  // and the FCM did not). Enabled per default ensures all three
  // algorithms see the same filtered input.
  bool apply_white_filter = true;
  int white_filter_max_width = 20;

  // ROI L y ROI R (en imagen IPM 320x240)
  // x_start, y_start, width, height
  int roi_left_x = 0;
  int roi_left_y = 0;
  int roi_left_width = 160;
  int roi_left_height = 240;

  int roi_right_x = 160;
  int roi_right_y = 0;
  int roi_right_width = 160;
  int roi_right_height = 240;
};

/**
 * @brief Resultado del preprocesado.
 *
 * Contiene la imagen original redimensionada (para visualización),
 * la imagen IPM, la imagen binaria de bordes (Canny),
 * y la matriz inversa del IPM (para proyectar de vuelta a perspectiva).
 */
struct PreprocessResult
{
  cv::Mat resized_bgr;        // 320x240 BGR (debug)
  cv::Mat ipm_bgr;            // 320x240 BGR tras IPM
  cv::Mat edges;              // 320x240 8UC1 binaria (Canny)
  cv::Mat inverse_ipm;        // 3x3 matriz para proyectar de IPM a perspectiva
};

/**
 * @brief Preprocesador compartido entre FCM y Sliding Windows.
 *
 * Encapsula resize, IPM, threshold, Canny y guarda la matriz inversa
 * para overlay. Garantiza que ambos algoritmos reciban exactamente
 * la misma imagen binaria de entrada.
 */
class Preprocessor
{
public:
  explicit Preprocessor(const PreprocessorParams & params = PreprocessorParams());

  /**
   * @brief Procesa una imagen BGR cruda y devuelve la salida binaria.
   * @param input Imagen BGR (típicamente 640x480 del simulador).
   * @return Resultado completo del preprocesado.
   */
  PreprocessResult process(const cv::Mat & input);

  /// Devuelve el ROI izquierdo como cv::Rect (en coordenadas IPM).
  cv::Rect getLeftRoi() const;

  /// Devuelve el ROI derecho como cv::Rect (en coordenadas IPM).
  cv::Rect getRightRoi() const;

  /// Devuelve los parámetros activos (para debug/log).
  const PreprocessorParams & params() const {return params_;}

private:
  PreprocessorParams params_;

  /// Suppress continuous runs of nonzero pixels wider than width_max
  /// pixels per row, in-place. Idempotent across rows. Static because
  /// it does not depend on instance state.
  static void applyWhiteFilter(cv::Mat & img, int width_max);
};

}  // namespace lane_common

#endif  // LANE_COMMON__PREPROCESSOR_HPP_
