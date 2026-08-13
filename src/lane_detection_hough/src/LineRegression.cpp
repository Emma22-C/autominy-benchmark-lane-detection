// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_hough/LineRegression.hpp"

namespace lane_detection_hough
{

LineModel fitLineSimple(
  const std::vector<cv::Point> & points,
  int min_points)
{
  LineModel out;
  if (static_cast<int>(points.size()) < min_points) {
    return out;
  }
  cv::Vec4f line;
  cv::fitLine(points, line, cv::DIST_L2, 0, 0.01, 0.01);
  out.vx = line[0];
  out.vy = line[1];
  out.x0 = line[2];
  out.y0 = line[3];
  out.valid = true;
  return out;
}

}  // namespace lane_detection_hough
