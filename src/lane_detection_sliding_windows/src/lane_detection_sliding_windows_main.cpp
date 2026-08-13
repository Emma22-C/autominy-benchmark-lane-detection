// Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
// Licensed under the Apache License, Version 2.0 (the "License").

#include "lane_detection_sliding_windows/LaneDetectionSlidingWindowsNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<lane_detection_sliding_windows::LaneDetectionSlidingWindowsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
