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
#include "std_msgs/msg/float32.hpp"


using namespace std::chrono_literals;
namespace v2v {

class V2V : public rclcpp::Node
{
public:
    V2V();

private:
    bool emergency_flag = true;
    float target_distance = 0.5f;
    float current_velocity = 0.0f;
    std::string truck_name;
    std::string to_;
  //Subscriber
  rclcpp::Subscription<ros2_msg::msg::Target>::SharedPtr TargetSubscriber_;
  rclcpp::Subscription<ros2_msg::msg::Target>::SharedPtr V2VSubscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr VelocitySubscriber;
  void TargetSubCallback(const ros2_msg::msg::Target::SharedPtr msg);
  void V2VSubCallback(const ros2_msg::msg::Target::SharedPtr msg);
  void timerCallback();
  void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
  //Publisher
  rclcpp::Publisher<ros2_msg::msg::Target>::SharedPtr TargetPublisher_;
  rclcpp::Publisher<ros2_msg::msg::Target>::SharedPtr V2VPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}