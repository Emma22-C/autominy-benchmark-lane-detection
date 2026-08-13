// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_common/Preprocessor.hpp"
#include <stdexcept>

namespace lane_common
{

Preprocessor::Preprocessor(const PreprocessorParams & params)
: params_(params)
{
}

PreprocessResult Preprocessor::process(const cv::Mat & input)
{
  if (input.empty()) {
    throw std::invalid_argument("Preprocessor::process: input image is empty");
  }

  PreprocessResult result;

  // 1. Resize (640x480 -> 320x240 con factor 0.5)
  cv::resize(
    input, result.resized_bgr,
    cv::Size(
      static_cast<int>(input.cols * params_.resize_factor),
      static_cast<int>(input.rows * params_.resize_factor)),
    0, 0, cv::INTER_LINEAR);

  // 2. IPM (Inverse Perspective Mapping) - homografía hacia bird-eye
  cv::Mat lambda = cv::getPerspectiveTransform(
    params_.ipm_src.data(),
    params_.ipm_dst.data());

  cv::warpPerspective(
    result.resized_bgr, result.ipm_bgr, lambda,
    cv::Size(result.resized_bgr.cols, result.resized_bgr.rows));

  // 3. Guardar inversa para overlay
  cv::invert(lambda, result.inverse_ipm);

  // 4. Threshold + Canny (idéntico al del paper)
  cv::Mat gray;
  cv::cvtColor(result.ipm_bgr, gray, cv::COLOR_BGR2GRAY);
  cv::medianBlur(gray, gray, params_.median_blur_kernel);
  cv::inRange(
    gray,
    params_.gray_threshold_low,
    params_.gray_threshold_high,
    gray);
  cv::Canny(
    gray, result.edges,
    params_.canny_low,
    params_.canny_high,
    params_.canny_aperture,
    false);

  // 5. White Filter (optional) - removes wide horizontal white runs.
  // Applied AFTER Canny, replicating the order in the original
  // lane_detectionslide.cpp (white_filter(gray, 16) after the Canny
  // call). Even though Canny output is "edges" rather than solid
  // blobs, in practice crosswalk-style patterns leave behind
  // closely-spaced horizontal edge runs that this filter clears out.
  if (params_.apply_white_filter) {
    applyWhiteFilter(result.edges, params_.white_filter_max_width);
  }

  return result;
}

void Preprocessor::applyWhiteFilter(cv::Mat & img, int width_max)
{
  // Direct port of white_filter() from lane_detectionslide.cpp.
  // For each row, find continuous runs of nonzero pixels; if a run
  // is wider than width_max, zero it out. Lane stripes (4-8 px wide)
  // pass through; crosswalk blocks (20-40 px wide) get suppressed.
  for (int r = 0; r < img.rows; ++r) {
    uchar * pixel = img.ptr<uchar>(r);
    uchar last_point = 0;
    int start = -1;
    int count_white = 0;

    for (int c = 0; c < img.cols; ++c) {
      const uchar now_point = pixel[c];
      if (last_point == 0 && now_point > 0) {
        start = c;
        count_white = 1;
      } else if (now_point > 0) {
        ++count_white;
      } else if (last_point > 0 && now_point == 0 && start != -1) {
        if (count_white >= width_max) {
          for (int k = start; k < c; ++k) {
            pixel[k] = 0;
          }
        }
        start = -1;
        count_white = 0;
      }
      last_point = now_point;
    }
    // Edge case: if the run reached the end of the row without
    // closing, also check it.
    if (start != -1 && count_white >= width_max) {
      for (int k = start; k < img.cols; ++k) {
        pixel[k] = 0;
      }
    }
  }
}

cv::Rect Preprocessor::getLeftRoi() const
{
  return cv::Rect(
    params_.roi_left_x,
    params_.roi_left_y,
    params_.roi_left_width,
    params_.roi_left_height);
}

cv::Rect Preprocessor::getRightRoi() const
{
  return cv::Rect(
    params_.roi_right_x,
    params_.roi_right_y,
    params_.roi_right_width,
    params_.roi_right_height);
}

}  // namespace lane_common
