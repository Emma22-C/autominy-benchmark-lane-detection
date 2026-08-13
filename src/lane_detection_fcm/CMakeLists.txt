cmake_minimum_required(VERSION 3.8)
project(lane_detection_fcm)

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

# Librería del algoritmo (sin dependencias ROS para facilitar tests)
add_library(fcm_algorithm SHARED
  src/FuzzyCMeans.cpp
  src/ClusterMetrics.cpp
)

target_include_directories(fcm_algorithm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(fcm_algorithm
  ${OpenCV_LIBS}
)

# Nodo ROS2
add_executable(lane_detection_fcm_node
  src/LaneDetectionFcmNode.cpp
  src/lane_detection_fcm_main.cpp
)

target_include_directories(lane_detection_fcm_node PRIVATE
  include
)

target_link_libraries(lane_detection_fcm_node
  fcm_algorithm
  ${OpenCV_LIBS}
)

ament_target_dependencies(lane_detection_fcm_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  lane_common
)

install(TARGETS
  lane_detection_fcm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  fcm_algorithm
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
