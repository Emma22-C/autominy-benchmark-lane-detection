// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_sliding_windows/SlidingWindows.hpp"
#include <algorithm>

namespace lane_detection_sliding_windows
{

SlidingWindows::SlidingWindows(const SlidingWindowsParams & params)
: params_(params)
{
}

std::pair<int, int> SlidingWindows::histogramPeaks(const cv::Mat & edges) const
{
  // Replica del Histogram() original: usa la franja inferior (filas 2/3 a 3/3-1)
  // como base para el histograma vertical.
  const int rows = edges.rows;
  const int cols = edges.cols;

  const int init_row = rows * 2 / 3;
  const int strip_height = std::max(rows / 3 - 1, 1);

  std::vector<int> hist(cols, 0);
  for (int c = 0; c < cols; ++c) {
    int sum_white = 0;
    for (int r = init_row; r < std::min(init_row + strip_height, rows); ++r) {
      if (edges.at<uchar>(r, c) > 0) {
        sum_white += 1;
      }
    }
    hist[c] = sum_white;
  }

  // Pico izquierdo: max en la mitad izquierda; derecho: max en la mitad derecha.
  const int mid = cols / 2;
  auto left_it = std::max_element(hist.begin(), hist.begin() + mid);
  auto right_it = std::max_element(hist.begin() + mid + 1, hist.end());

  const int left_x = static_cast<int>(std::distance(hist.begin(), left_it));
  const int right_x = static_cast<int>(std::distance(hist.begin(), right_it));

  return {left_x, right_x};
}

SlidingWindowsResult SlidingWindows::locate(const cv::Mat & edges)
{
  SlidingWindowsResult out;

  if (edges.empty() || edges.type() != CV_8UC1) {
    return out;
  }

  const int rows = edges.rows;
  const int cols = edges.cols;

  const auto [left_start, right_start] = histogramPeaks(edges);
  out.left.starting_x = left_start;
  out.right.starting_x = right_start;

  const int window_height = std::max(rows / params_.n_windows, 1);

  int leftx_current = left_start;
  int rightx_current = right_start;

  for (int w = 0; w < params_.n_windows; ++w) {
    int y_low = rows - (w + 1) * window_height;
    int y_high = rows - w * window_height;
    int xL_low = leftx_current - params_.margin;
    int xL_high = leftx_current + params_.margin;
    int xR_low = rightx_current - params_.margin;
    int xR_high = rightx_current + params_.margin;

    // Recortar a los límites
    xL_low = std::max(xL_low, 1);
    xR_high = std::min(xR_high, cols - 1);
    y_low = std::max(y_low, 1);
    y_high = std::min(y_high, rows - 1);

    int mean_leftx = 0;
    int mean_rightx = 0;
    int count_left = 0;
    int count_right = 0;

    for (int r = y_low; r < y_high; ++r) {
      for (int c = xL_low + 1; c < xL_high; ++c) {
        if (edges.at<uchar>(r, c) > 0) {
          out.left.points.emplace_back(c, r);
          mean_leftx += c;
          ++count_left;
        }
      }
      for (int c = xR_low + 1; c < xR_high; ++c) {
        if (edges.at<uchar>(r, c) > 0) {
          out.right.points.emplace_back(c, r);
          mean_rightx += c;
          ++count_right;
        }
      }
    }

    // Reajustar centros si tenemos suficientes pixels
    if (count_left >= params_.min_pixels_per_window) {
      leftx_current = mean_leftx / count_left;
    }
    if (count_right >= params_.min_pixels_per_window) {
      rightx_current = mean_rightx / count_right;
    }

    out.left.windows.emplace_back(xL_low, y_low, xL_high - xL_low, y_high - y_low);
    out.right.windows.emplace_back(xR_low, y_low, xR_high - xR_low, y_high - y_low);
  }

  return out;
}

}  // namespace lane_detection_sliding_windows
