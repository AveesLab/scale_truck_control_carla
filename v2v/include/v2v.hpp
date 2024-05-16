#pragma once

// C++
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <sys/time.h>
#include <algorithm>
#include <limits>
#include <random>
#include <condition_variable>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/cmd2xav.hpp"
#include "ros2_msg/msg/obj2xav.hpp"
#include "ros2_msg/msg/target.hpp"
#include "ros2_msg/msg/xav2cmd.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"


namespace v2v {

class V2V : public rclcpp::Node
{
public:
    V2V();
    int number;

private:
    bool emergency_flag = true;
    float target_distance = 14.0f;
    float target_velocity = 0.0f;
    float current_distance = 14.0f;
    float cur_vel_ = 0.0;
    std::string truck_name;
    std::string to_;
  //Subscriber
  rclcpp::Subscription<ros2_msg::msg::Target>::SharedPtr TargetSubscriber_;
  rclcpp::Subscription<ros2_msg::msg::Target>::SharedPtr V2VSubscriber_;
  rclcpp::Subscription<ros2_msg::msg::Obj2xav>::SharedPtr DistanceSubscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr VelocitySubscriber;

  void DistanceSubCallback(const ros2_msg::msg::Obj2xav::SharedPtr msg);
  void TargetSubCallback(const ros2_msg::msg::Target::SharedPtr msg);
  void V2VSubCallback(const ros2_msg::msg::Target::SharedPtr msg);
  void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
  //Publisher
  rclcpp::Publisher<ros2_msg::msg::Target>::SharedPtr TargetPublisher_;
  rclcpp::Publisher<ros2_msg::msg::Target>::SharedPtr V2VPublisher_;
  //rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr InfoPublisher_;
  rclcpp::Publisher<ros2_msg::msg::Xav2cmd>::SharedPtr InfoPublisher_;
};

}
