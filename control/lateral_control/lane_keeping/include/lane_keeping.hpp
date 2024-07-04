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
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "spline.h"

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/xav2lane.hpp"
#include "ros2_msg/msg/lane2xav.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"

#define _GUN_SOURCE

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

namespace lanekeeping {
  
  class LaneKeeping : public rclcpp::Node
  {
  public:
    LaneKeeping();
    //Timer
    void get_steer_coef(float vel);
    float K1_, K2_,K3_,K4_;
    float cur_vel_ = 0.0f;
    void controlSteer();
    ros2_msg::msg::Lane2xav lane_coef_, poly_coef_;
    Mat left;
    Mat right;
    Mat center;
    vector<Mat> line_;
    std_msgs::msg::Float32 steer_;
    int current_center = 2; // or 4(right)
    bool lc_right_flag_ = false;
    bool lc_left_flag_ = false;
    bool lane_keeping = true;
    int right_cnt = 0;
    int left_cnt = 0;
  private:
    void LoadParams(void);
  
    //Publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr SteerPublisher_;
    //Subscriber
    rclcpp::Subscription<ros2_msg::msg::Xav2lane>::SharedPtr XavSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Lane2xav>::SharedPtr LaneSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr LaneChangeSubscriber_;
    //Callback Func
    void XavSubCallback(const ros2_msg::msg::Xav2lane::SharedPtr msg);
    void LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg);
    void LaneChangeSubCallback(const std_msgs::msg::Int32::SharedPtr msg);
    tk::spline cspline(int mark_);
    float SteerAngle_;
    float eL_height_, e1_height_, lp_, trust_height_;
    float K_;
    double a_[5], b_[5];
    vector<float> e_values_;
  
  };

}
