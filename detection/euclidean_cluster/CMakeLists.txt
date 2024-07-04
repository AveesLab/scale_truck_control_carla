cmake_minimum_required(VERSION 3.5.0)

project(euclidean_cluster)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(ros2_msg REQUIRED)

set(PROJECT_SOURCES
  src/euclidean_cluster.cpp
)

include_directories(include ${cv_bridge_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(euclidean_cluster_node ${PROJECT_SOURCES})

ament_target_dependencies(euclidean_cluster_node
    "rclcpp"
    "sensor_msgs"
    "std_msgs"
    "cv_bridge"
    "OpenCV"
    "pcl_conversions"
    "PCL"
    geometry_msgs
    pcl_ros
    shape_msgs
    tf2_ros
    ros2_msg
)

target_link_libraries(euclidean_cluster_node
  ${PCL_LIBRARIES}
  pcl_common
  pcl_io
  pcl_filters
  pcl_kdtree
  pcl_segmentation
)

install(TARGETS 
  euclidean_cluster_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  laser_geometry
)
ament_package()
