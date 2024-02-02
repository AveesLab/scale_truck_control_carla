cmake_minimum_required(VERSION 3.5.0)

project(object_detection_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(ros2_msg REQUIRED)

include_directories(include ${cv_bridge_INCLUDE_DIRS})

add_library(object_detection_ros2 SHARED
  src/pcl_object_detection.cpp)
target_compile_definitions(object_detection_ros2
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(object_detection_ros2
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
  visualization_msgs
  ros2_msg
)

rclcpp_components_register_nodes(object_detection_ros2 "object_detection_ros2::PclObjectDetection")
set(node_plugins "${node_plugins}object_detection_ros2::PclObjectDetection;$<TARGET_FILE:object_detection_ros2>\n")

add_executable(object_detection_ros2_node
    src/pcl_object_detection_node.cpp)
target_link_libraries(object_detection_ros2_node
    object_detection_ros2
    laser_geometry::laser_geometry
    
)
    
ament_target_dependencies(object_detection_ros2_node
    "rclcpp"
)


install(TARGETS
    object_detection_ros2
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


install(TARGETS
    object_detection_ros2_node
    DESTINATION lib/${PROJECT_NAME}
)


# Install launch files.
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

#install(DIRECTORY include
#    DESTINATION include/${PROJECT_NAME}
#)

ament_export_dependencies(
  laser_geometry
)
ament_package()
