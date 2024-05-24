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
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/xav2lane.hpp"
#include "ros2_msg/msg/lane2xav.hpp"
#include "ros2_msg/msg/obj2xav.hpp"
#include "ros2_msg/msg/target.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

namespace Planner {
using namespace std::chrono_literals;
class Planner : public rclcpp::Node
{
public:
    Planner();
private:
    bool emergency_flag = true;
    float stop_dist = 3.0f;
    std::string truck_name;
    bool lv = false;
    bool emergency_brake = false;
    float Kp_dist_ = 0.8; // 2.0; //0.8;
    float Kd_dist_ = 0.03; //0.05;
    float target_velocity = 0.0f;
    float target_distance = 14.0f;
    float current_distance = 0.0f;
    float dt_ = 0.01f;
    float des_spacing = 0.0f;
    float spacing_err = 0.0f;
    float speed_err = 0.0f;
    float myGapControlGainGap = 0.45f;
    float myGapControlGainGapDot = 0.35f;
    float current_velocity = 0.0f;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr DistanceSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr VelocitySubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Target>::SharedPtr TargetSubscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr EmergencyPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr TargetVelocityPublisher_;

    void timerCallback();
    float calculate_target_velocity();
    void send_full_brake();
    bool check_collision();
    void DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void TargetSubCallback(const ros2_msg::msg::Target::SharedPtr msg);
    void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void calculate_cacc_param();
    float calculate_target_velocity_cacc();
};

}