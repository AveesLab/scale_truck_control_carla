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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/xav2lane.hpp"
#include "ros2_msg/msg/lane2xav.hpp"
#include "ros2_msg/msg/obj2xav.hpp"
#include "ros2_msg/msg/target.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ros2_msg/msg/fusing_array.hpp"
#include "ros2_msg/msg/tracking_array.hpp"
#include "ros2_msg/msg/v2_xcam.hpp"
#include "ros2_msg/msg/v2_xcustom.hpp"


typedef struct object_info {
    float x, y, w, h, distance, velocity;
    int class_id;
    int id; // tracker_id
} object_info;

struct BoundingBox {
    double x; // x-coordinate of the top-left corner
    double y; // y-coordinate of the top-left corner
    double w; // width of the bounding box
    double h; // height of the bounding box
};
using namespace cv;
namespace Planner {
using namespace std::chrono_literals;


class Planner : public rclcpp::Node
{
public:
    Planner();
private:
    std::vector<object_info> detected_objects;
    std::vector<object_info> detected_objects_on_ego_lane;
    ros2_msg::msg::Lane2xav poly_coef_;
    int cur_ego_lane  = 2;
    bool is_target_trailer = false;
    int preceding_truck_id = 0;
    std::vector<Mat> line_;
    bool emergency_flag = false; //test
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
    int lc_mode= 0;
    float des_spacing = 0.0f;
    float spacing_err = 0.0f;
    float speed_err = 0.0f;
    float myGapControlGainGap = 0.45f;
    float myGapControlGainGapDot = 0.35f;
    float current_velocity = 0.0f;
    bool cut_in_flag = false;
    int ulane_change = 0;
    bool emergency_flag_from = false;
    bool caution1mode = false;
    bool right_obstacle = false;
    bool left_obstacle = false;
    bool received_ = false;
    bool detected_objects_ = false;
    bool left_object = false; 
    bool right_object = false;
    bool lane_ = false;
    bool sync_ = false;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr DistanceSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr VelocitySubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Target>::SharedPtr TargetSubscriber_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr EmergencyPublisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr CutinFlagPublisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr caution1Publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr lcPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr TargetVelocityPublisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr DistancePublisher_;
    rclcpp::Subscription<ros2_msg::msg::FusingArray>::SharedPtr DetectedObjectsSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::Lane2xav>::SharedPtr LaneSubscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr uLaneSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::V2XCAM>::SharedPtr V2xcamSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::V2XCUSTOM>::SharedPtr V2xcustomSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr LeftObjectSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr RightObjectSubscriber_;
    void timerCallback();
    float calculate_target_velocity();
    void send_full_brake();
    bool check_collision();
    void DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void TargetSubCallback(const ros2_msg::msg::Target::SharedPtr msg);
    void DetectedObjectsSubcallback(const ros2_msg::msg::FusingArray::SharedPtr msg);
    void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void LaneSubcallback(const ros2_msg::msg::Lane2xav::SharedPtr msg);
    void uLaneSubcallback(const std_msgs::msg::Int32::SharedPtr msg);
    void calculate_cacc_param();
    float calculate_target_velocity_cacc();
    void check_objects_ego_lane();
    void sort_objects_by_distance(std::vector<object_info>& objects);
    void register_trailer_to_follow();
    bool isInEgoLane(const BoundingBox& box, double a, double b, double c, double ar, double br, double cr);
    float calculate_target_velocity_acc();
    void calculate_acc_param();
    float check_ttc();
    void check_ego_lane_num();
    void lane_change_flag(int num);
    void CAMSubcallback(const ros2_msg::msg::V2XCAM::SharedPtr msg);
    void CUSTOMSubcallback(const ros2_msg::msg::V2XCUSTOM::SharedPtr msg);
    bool check_side(int num);
    void LeftObjectSubcallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg); 
    void RightObjectSubcallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    bool data_received();
};

}