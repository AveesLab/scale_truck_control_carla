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



namespace interface {

class interface : public rclcpp::Node
{
public:
    interface();

private:
    bool emergency_flag = true;
    float target_distance = 14.0f;
    float target_velocity = 0.0f;
    
  //Subscriber
  rclcpp::Subscription<ros2_msg::msg::Cmd2xav>::SharedPtr CmdSubscriber_;
  void CmdSubCallback(const ros2_msg::msg::Cmd2xav::SharedPtr msg);
  //Publisher
  rclcpp::Publisher<ros2_msg::msg::Target>::SharedPtr TargetPublisher_;
};

}