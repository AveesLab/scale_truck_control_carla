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
#include "ros2_msg/msg/target.hpp"



namespace v2v {

class V2V : public rclcpp::Node
{
public:
    V2V();

private:
    bool emergency_flag = true;
    float target_distance = 14.0f;
    float target_velocity = 0.0f;
    std::string truck_name;
    std::string to_;
  //Subscriber
  rclcpp::Subscription<ros2_msg::msg::Target>::SharedPtr TargetSubscriber_;
  rclcpp::Subscription<ros2_msg::msg::Target>::SharedPtr V2VSubscriber_;
  void TargetSubCallback(const ros2_msg::msg::Target::SharedPtr msg);
  void V2VSubCallback(const ros2_msg::msg::Target::SharedPtr msg);
  //Publisher
  rclcpp::Publisher<ros2_msg::msg::Target>::SharedPtr TargetPublisher_;
  rclcpp::Publisher<ros2_msg::msg::Target>::SharedPtr V2VPublisher_;
};

}