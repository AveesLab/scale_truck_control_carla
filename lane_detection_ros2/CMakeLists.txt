cmake_minimum_required(VERSION 3.8)
project(lane_detection_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED PATHS "/usr/include/opencv4")
find_package(cv_bridge REQUIRED)
find_package(ros2_msg REQUIRED)

set(PROJECT_SOURCES
  src/main.cpp
  src/lane_detect.cpp

  include/lane_detect.hpp
)

set(PROJECT_SOURCES_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  ament_index_cpp
  cv_bridge
  ros2_msg
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_detect_node ${PROJECT_SOURCES})
ament_target_dependencies(lane_detect_node ${PROJECT_SOURCES_DEPENDENCIES})

install(TARGETS 
  lane_detect_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
