// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_hough/HoughLaneExtractor.hpp"
#include <cmath>

namespace lane_detection_hough
{

HoughLaneExtractor::HoughLaneExtractor(const HoughParams & params)
: params_(params)
{
}

bool HoughLaneExtractor::isVerticalEnough(const cv::Vec4i & seg) const
{
  // seg = (x1, y1, x2, y2). Queremos descartar segmentos casi-horizontales.
  // En IPM las líneas de carril deberían ser verticales o cuasi-verticales.
  const int dx = seg[2] - seg[0];
  const int dy = seg[3] - seg[1];
  if (dy == 0) {
    return false;  // perfectamente horizontal -> descartar
  }
  // |dy / dx| debe ser grande (vertical). Equivalente: |dx / dy| <
  // 1/min_abs_slope. Definimos el filtro de manera directa.
  const double abs_slope = std::fabs(
    static_cast<double>(dy) /
    static_cast<double>(dx == 0 ? 1 : dx));
  // Si dx == 0 (vertical perfecta), pasa siempre.
  if (dx == 0) {
    return true;
  }
  return abs_slope >= params_.min_abs_slope;
}

HoughResult HoughLaneExtractor::extract(const cv::Mat & edges, int midline_x)
{
  HoughResult out;

  if (edges.empty() || edges.type() != CV_8UC1) {
    return out;
  }

  // 1. Hough probabilístico
  std::vector<cv::Vec4i> segments;
  cv::HoughLinesP(
    edges, segments,
    params_.rho_resolution,
    params_.theta_resolution,
    params_.votes_threshold,
    params_.min_line_length,
    params_.max_line_gap);

  // 2 + 3. Filtro de pendiente + clasificación L/R
  for (const auto & seg : segments) {
    if (!isVerticalEnough(seg)) {
      continue;
    }
    // Clasificar por X del punto medio
    const double mid_x = 0.5 * (seg[0] + seg[2]);

    HoughLineResult * target = nullptr;
    if (mid_x < static_cast<double>(midline_x)) {
      target = &out.left;
    } else {
      target = &out.right;
    }
    target->raw_segments.push_back(seg);
    target->endpoints.emplace_back(seg[0], seg[1]);
    target->endpoints.emplace_back(seg[2], seg[3]);
  }

  return out;
}

}  // namespace lane_detection_hough
