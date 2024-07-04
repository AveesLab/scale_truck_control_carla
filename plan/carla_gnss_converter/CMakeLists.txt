cmake_minimum_required(VERSION 3.5)
project(carla_gnss_converter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ros2_msg REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gnss_converter src/gnss_converter.cpp)
ament_target_dependencies(gnss_converter rclcpp sensor_msgs geometry_msgs ros2_msg message_filters std_msgs)

target_include_directories(gnss_converter PRIVATE ${EIGEN3_INCLUDE_DIR})

install(TARGETS
  gnss_converter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
