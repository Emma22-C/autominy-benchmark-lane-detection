// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef LANE_DETECTION_HOUGH__HOUGH_LANE_EXTRACTOR_HPP_
#define LANE_DETECTION_HOUGH__HOUGH_LANE_EXTRACTOR_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace lane_detection_hough
{

/**
 * @brief Parámetros del extractor basado en Hough.
 *
 * Hough probabilístico (cv::HoughLinesP) opera directamente sobre la
 * imagen binaria del Canny y devuelve un conjunto de segmentos de
 * línea (x1,y1,x2,y2). Para detección de carril en IPM:
 *
 * - rho_resolution: 1 px típicamente.
 * - theta_resolution: pi/180 (1 grado).
 * - threshold: voto mínimo para considerar una línea (depende del
 *   tamaño de la imagen y la cantidad esperada de píxeles por línea).
 * - min_line_length: longitud mínima en píxeles para un segmento.
 * - max_line_gap: gap máximo permitido entre puntos del mismo segmento.
 * - min_slope_abs: pendiente absoluta mínima |dx/dy| para descartar
 *   segmentos casi-horizontales (no son carril en IPM).
 *
 * Valores por defecto razonables para imagen IPM 320x240.
 */
struct HoughParams
{
  double rho_resolution = 1.0;
  double theta_resolution = CV_PI / 180.0;  // 1 deg
  int votes_threshold = 25;
  double min_line_length = 20.0;
  double max_line_gap = 10.0;
  // Filtro de pendiente: descartar segmentos casi horizontales.
  // En IPM las líneas del carril son verticales o cuasi-verticales.
  // |dy/dx| > 1.0 significa "más vertical que horizontal".
  // Usamos |dy/(dx+eps)| para evitar div/0 y exigir min 0.5.
  double min_abs_slope = 0.5;
};

/**
 * @brief Resultado del extractor para un lado del carril.
 */
struct HoughLineResult
{
  /// Segmentos crudos detectados por Hough en este lado.
  std::vector<cv::Vec4i> raw_segments;
  /// Endpoints de los segmentos (para alimentar la regresión).
  std::vector<cv::Point> endpoints;
};

struct HoughResult
{
  HoughLineResult left;
  HoughLineResult right;
};

/**
 * @brief Detector de carriles basado en Hough probabilístico.
 *
 * Flujo:
 *  1. cv::HoughLinesP sobre la imagen binaria preprocesada.
 *  2. Filtrar segmentos por pendiente (descartar horizontales).
 *  3. Clasificar cada segmento en izquierda/derecha por la X del
 *     punto medio (usando una línea divisoria vertical en cols/2).
 *  4. Extraer los endpoints de los segmentos filtrados.
 *
 * La regresión la hace LineRegression sobre los endpoints. Esta clase
 * solo hace la detección de segmentos y la clasificación L/R.
 */
class HoughLaneExtractor
{
public:
  HoughLaneExtractor() = default;
  explicit HoughLaneExtractor(const HoughParams & params);

  /**
   * @brief Ejecuta el pipeline Hough sobre la imagen binaria.
   *
   * @param edges Imagen 8UC1 (salida del Canny del lane_common::Preprocessor).
   * @param midline_x Columna que divide L/R (típicamente edges.cols/2).
   * @return Segmentos y endpoints separados por lado.
   */
  HoughResult extract(const cv::Mat & edges, int midline_x);

  const HoughParams & params() const {return params_;}

private:
  HoughParams params_;

  /// True si el segmento pasa el filtro de pendiente mínima vertical.
  bool isVerticalEnough(const cv::Vec4i & seg) const;
};

}  // namespace lane_detection_hough

#endif  // LANE_DETECTION_HOUGH__HOUGH_LANE_EXTRACTOR_HPP_
