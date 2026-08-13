cmake_minimum_required(VERSION 3.8)
project(lane_detection_hough)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(lane_common REQUIRED)

add_library(hough_algorithm SHARED
  src/HoughLaneExtractor.cpp
  src/LineRegression.cpp
)

target_include_directories(hough_algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(hough_algorithm
  ${OpenCV_LIBS}
)

add_executable(lane_detection_hough_node
  src/LaneDetectionHoughNode.cpp
  src/lane_detection_hough_main.cpp
)

target_include_directories(lane_detection_hough_node PRIVATE
  include
)

target_link_libraries(lane_detection_hough_node
  hough_algorithm
  ${OpenCV_LIBS}
)

ament_target_dependencies(lane_detection_hough_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  lane_common
)

install(TARGETS
  lane_detection_hough_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  hough_algorithm
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
