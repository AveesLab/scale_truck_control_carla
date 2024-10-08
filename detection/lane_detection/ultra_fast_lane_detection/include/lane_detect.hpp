#pragma once

#include "cuda_runtime_api.h"
#include "logging.h"
#include "common.hpp"
#include "macros.h"
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

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <opencv2/opencv.hpp>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/xav2lane.hpp"
#include "ros2_msg/msg/lane2xav.hpp"
#include "ros2_msg/msg/obj2xav.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"

#define _GUN_SOURCE

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

namespace LaneDetect {

class LaneDetector : public rclcpp::Node
{
public:
  LaneDetector();
  ~LaneDetector();

  //Timer
  struct timeval start_, end_;

  float display_img(Mat _frame, int _delay, bool _view);
  int distance_ = 0;
  Mat frame_;
  float rotation_angle_ = 0.0f;
  float lateral_offset_ = 0.0f;
  Point left_, right_;
  float y_offset_ = 0.0f;

  /********** bbox *********/
  std::string name_;
  std::string r_name_;
  unsigned int x_ = 0, y_ = 0, w_ = 0, h_ = 0;
  unsigned int rx_ = 0, ry_ = 0, rw_ = 0, rh_ = 0;
  Point center_, warp_center_;
  float log_e1_ = 0.0f;
  float log_el_ = 0.0f;
  float vel_ = 0.0f;
     int fcount = 0;
    int vis_h = 480;
    int vis_w = 640;
    int col_sample_w = 8;
    char *trtModelStream{ nullptr };
    size_t size{ 0 };
    IRuntime* runtime;
    ICudaEngine* engine;
    IExecutionContext* context;
    std::vector<int> tusimple_row_anchor
            { 126, 132, 138, 144, 150, 156, 162, 168, 174, 180, 186, 192, 198, 204, 210, 216, 222, 228, 234, 240, 246, 252, 258, 264, 270, 276, 282};
  Mat left_coef_;
  Mat right_coef_;
  Mat center_coef_;
  Mat extra_right_coef_;
  Mat extra_center_coef_;
  Mat orig_left_coef_;
  Mat orig_right_coef_;
  Mat orig_extra_right_coef_;
private:
  void LoadParams(void);
  void doInference(IExecutionContext& context, float* input, float* output, int batchSize);
  void softmax_mul(float* x, float* y, int rows, int cols, int chan);
  void argmax(float* x, float* y, int rows, int cols, int chan);
  Mat polyfit(vector<int> x_val, vector<int> y_val);
  void get_lane_coef();
  void clear_release();

  cv::Point2f transformPoint(const cv::Point& pt, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs); 

  //Publisher
  rclcpp::Publisher<ros2_msg::msg::Lane2xav>::SharedPtr XavPublisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr SteerPublisher_;
  //Subscriber
  rclcpp::Subscription<ros2_msg::msg::Xav2lane>::SharedPtr XavSubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ImageSubscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr DistanceSubscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ShutdownSubscriber;
  //Callback Func
  void XavSubCallback(const ros2_msg::msg::Xav2lane::SharedPtr msg);
  void ImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void shutdown_callback(const std_msgs::msg::String::SharedPtr msg);
  bool viewImage_;
  int waitKeyDelay_;
  bool droi_ready_ = false;
  bool isNodeRunning_ = true;
  bool controlDone_ = false;
  ros2_msg::msg::Lane2xav lane_coef_;
  int center_select_ = 1;
  bool TEST = false;
  float frontRoi_ratio = 0.0f;
  float rearRoi_ratio = 0.0f;

  // lane change
  bool wroi_flag_ = false;
  bool L_flag = true;
  bool R_flag = true;
  bool E_flag = true;
  bool E2_flag = true;
  bool lc_center_follow_ = true;
  bool lc_right_flag = false;
  bool lc_right_flag_ = false;
  bool lc_left_flag = false;
  bool lc_left_flag_ = false;

  //image
  bool imageStatus_ = false;
  std_msgs::msg::Header imageHeader_;
  cv::Mat camImageCopy_;
  float AngleDegree_;
  cv::Mat prev_frame, prev2_frame;
  cv::Mat cluster_frame;

  //rear
  bool rear_view_ = false;
  bool rearImageStatus_ = false;
  std_msgs::msg::Header rearImageHeader_;
  cv::Mat rearCamImageCopy_;

  /*  Callback Synchronisation  */
  mutex cam_mutex;
  mutex dist_mutex_;
  bool cam_new_frame_arrived;
  condition_variable cam_condition_variable;


  std::thread lanedetect_Thread;
  void lanedetectInThread();

  /********** Camera calibration **********/
  Mat f_camera_matrix, f_dist_coeffs;
  Mat r_camera_matrix, r_dist_coeffs;
  Mat map1_, map2_, f_map1_, f_map2_, r_map1_, r_map2_;
  int canny_thresh1_, canny_thresh2_;

  /********** Lane_detect ***********/
  vector<Point2f> test_corners_, corners_, fROIcorners_, rROIcorners_, lROIcorners_, rearROIcorners_, testROIcorners_;
  vector<Point2f> test_warpCorners_, warpCorners_, fROIwarpCorners_, rROIwarpCorners_, lROIwarpCorners_, rearROIwarpCorners_, testROIwarpCorners_;

  int last_Llane_base_;
  int last_Rlane_base_;
  int last_Elane_base_;
  int last_E2lane_base_;

  float left_curve_radius_;
  float right_curve_radius_;
  float center_position_;
  float SteerAngle_;
  float SteerAngle2_;
  float eL_height_, trust_height_, e1_height_, lp_;
  float eL_height2_;
  float K_;
  double a_[5], b_[5];
  double a2_[3], b2_[3];
  vector<float> e_values_;
  float target_x_;
  float target_y_;

  vector<int> left_lane_inds_;
  vector<int> right_lane_inds_;
  vector<int> left_x_;
  vector<int> left_y_;
  vector<int> right_x_;
  vector<int> right_y_;
  vector<int> center_x_;
  vector<int> center_y_;

  /********** PID control ***********/
  int prev_lane_, prev_pid_;
  double Kp_term_, Ki_term_, Kd_term_, err_, prev_err_, I_err_, D_err_, result_;

  int width_, height_;
  bool option_; // dynamic ROI
  int threshold_;
  double diff_;

  int crop_x_, crop_y_, crop_width_, crop_height_;

  int Threshold_box_size_, Threshold_box_offset_;

  bool sync_with_delay = false;
};

}
