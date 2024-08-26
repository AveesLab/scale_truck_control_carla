cmake_minimum_required(VERSION 3.8)
project(test_fusion_lidar_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(ros2_msg REQUIRED)

set(PROJECT_SOURCES
  src/main.cpp
  src/test_fusion_lidar_node.cpp

  include/test_fusion_lidar_node.hpp
)

set(PROJECT_SOURCES_DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  ros2_msg
  ament_index_cpp
  cv_bridge
  message_filters
)


include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(test_fusion_lidar_node ${PROJECT_SOURCES})
ament_target_dependencies(test_fusion_lidar_node ${PROJECT_SOURCES_DEPENDENCIES})

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
)

install(TARGETS 
  test_fusion_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        include/
        DESTINATION include
)

install(DIRECTORY
        DESTINATION share/${PROJECT_NAME}
)

ament_package()