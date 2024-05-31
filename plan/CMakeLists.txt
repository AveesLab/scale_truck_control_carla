cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_msg REQUIRED)

set(PROJECT_SOURCES
  src/planner.cpp

  include/planner.hpp
)

set(PROJECT_SOURCES_DEPENDENCIES
  rclcpp
  std_msgs
  ament_index_cpp
  ros2_msg
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(planner_node ${PROJECT_SOURCES})
ament_target_dependencies(planner_node ${PROJECT_SOURCES_DEPENDENCIES})

install(TARGETS 
planner_node
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
