cmake_minimum_required(VERSION 3.8)
project(object_tracking_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILE_ID MATCHES "clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV)
find_package(ros2_msg REQUIRED)

set(PROJECT_SOURCES
    src/main.cpp
    src/object_tracking_node.cpp
    src/tracker.cpp
    src/track.cpp
    src/kalman_filter.cpp
    src/matrix.cpp
    src/munkres.cpp

    include/object_tracking_node.hpp
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

add_executable(object_tracking_ros2 ${PROJECT_SOURCES})
ament_target_dependencies(object_tracking_ros2 ${PROJECT_SOURCES_DEPENDENCIES})


install(DIRECTORY
    include/
    DESTINATION include
)

install(TARGETS
    object_tracking_ros2
    DESTINATION lib/${PROJECT_NAME}    
)


ament_package()