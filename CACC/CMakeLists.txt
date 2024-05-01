cmake_minimum_required(VERSION 3.8)
project(cacc_sumo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_msg REQUIRED)

set(PROJECT_SOURCES
  src/CACC_sumo_node.cpp
  include/CACC_sumo_node.hpp
)

set(PROJECT_SOURCES_DEPENDENCIES
  rclcpp
  std_msgs
  ros2_msg
)
include_directories(
  include
)
add_executable(cacc_sumo ${PROJECT_SOURCES})
ament_target_dependencies(cacc_sumo ${PROJECT_SOURCES_DEPENDENCIES})

install(TARGETS
cacc_sumo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
