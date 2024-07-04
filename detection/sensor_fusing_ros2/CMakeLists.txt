cmake_minimum_required(VERSION 3.8)
project(sensor_fusing_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ros2_msg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)


set(PROJECT_SOURCES
  src/main.cpp
  src/sensor_fusing_node.cpp

  include/sensor_fusing_node.hpp
)

set(PROJECT_SOURCES_DEPENDENCIES
    rclcpp
    std_msgs
    cv_bridge
    ros2_msg
    sensor_msgs
)


include_directories(
    include
)

add_executable(sensor_fusing_ros2 ${PROJECT_SOURCES})
ament_target_dependencies(sensor_fusing_ros2 ${PROJECT_SOURCES_DEPENDENCIES})


install(DIRECTORY
    include/
    DESTINATION include
)

install(TARGETS
    sensor_fusing_ros2
    DESTINATION lib/${PROJECT_NAME}    
)

ament_package()