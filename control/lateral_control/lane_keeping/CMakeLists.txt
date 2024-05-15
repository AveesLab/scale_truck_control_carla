cmake_minimum_required(VERSION 3.8)
project(lane_keeping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_msg REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(PROJECT_SOURCES
  src/lane_keeping.cpp
  include/lane_keeping.hpp
)

set(PROJECT_SOURCES_DEPENDENCIES
  rclcpp
  std_msgs
  ament_index_cpp
  cv_bridge
  ros2_msg
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_keeping_node ${PROJECT_SOURCES})
ament_target_dependencies(lane_keeping_node ${PROJECT_SOURCES_DEPENDENCIES})

install(TARGETS 
  lane_keeping_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
