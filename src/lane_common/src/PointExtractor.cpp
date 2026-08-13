// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_common/PointExtractor.hpp"

namespace lane_common
{

ExtractedPoints extractPoints(
  const cv::Mat & edges,
  const cv::Rect & roi_left,
  const cv::Rect & roi_right,
  int edge_margin)
{
  ExtractedPoints out;

  if (edges.empty() || edges.type() != CV_8UC1) {
    return out;
  }

  // Recorre cada ROI extrayendo píxeles >0.
  // Mantenemos el margen de borde (>=2 en el original) para evitar artefactos.
  auto scan_roi = [&](const cv::Rect & roi, std::vector<cv::Point> & out_pts) {
      const int x_start = std::max(roi.x + edge_margin, edge_margin);
      const int y_start = std::max(roi.y + edge_margin, edge_margin);
      const int x_end = std::min(roi.x + roi.width, edges.cols - edge_margin);
      const int y_end = std::min(roi.y + roi.height, edges.rows - edge_margin);

      for (int y = y_start; y < y_end; ++y) {
        const uchar * row = edges.ptr<uchar>(y);
        for (int x = x_start; x < x_end; ++x) {
          if (row[x] > 0) {
            // Guardamos (x,y) en coordenadas IPM absolutas.
            out_pts.emplace_back(x, y);
          }
        }
      }
    };

  scan_roi(roi_left, out.left);
  scan_roi(roi_right, out.right);

  return out;
}

}  // namespace lane_common
