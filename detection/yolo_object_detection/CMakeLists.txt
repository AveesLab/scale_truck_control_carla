cmake_minimum_required(VERSION 3.8)
project(yolo_object_detection_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DARKNET_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/darknet/include)
set(DARKNET_LIBRARY ${PROJECT_SOURCE_DIR}/darknet/libdarknet.so)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ros2_msg REQUIRED)

set(PROJECT_SOURCES
  src/main.cpp
  src/yolo_object_detection_node.cpp

  include/yolo_object_detection_node.hpp
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
    ${DARKNET_INCLUDE_DIR}
)

add_executable(yolo_object_detection_ros2 ${PROJECT_SOURCES})
ament_target_dependencies(yolo_object_detection_ros2 ${PROJECT_SOURCES_DEPENDENCIES})

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${DARKNET_LIBRARY}
)

install(TARGETS 
  yolo_object_detection_ros2
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
        launch
	config
	yolo_network_config
	darknet
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
