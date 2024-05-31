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
#include <mutex>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

#include <opencv2/opencv.hpp>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/lane2xav.hpp"
#include "std_msgs/msg/float32.hpp"

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

  Mat left_coef_;
  Mat right_coef_;
  Mat center_coef_;
private:
  void LoadParams(void);
  int arrMaxIdx(int hist[], int start, int end, int Max);
  std::vector<int> clusterHistogram(int* hist, int clusters);
  Mat polyfit(vector<int> x_val, vector<int> y_val);
  Mat detect_lines_sliding_window(Mat _frame, bool _view);
  Point warpPoint(Point center, Mat trans);
  float lowPassFilter(double sampling_time, float est_value, float prev_res);
  Mat draw_lane(Mat _sliding_frame, Mat _frame);
  Mat drawBox(Mat frame);
  void get_lane_coef();
  void clear_release();

  cv::Point2f transformPoint(const cv::Point& pt, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs); 

  //Publisher
  rclcpp::Publisher<ros2_msg::msg::Lane2xav>::SharedPtr LanePublisher_;
  //Subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ImageSubscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr DistanceSubscriber_;
  //Callback Func
  void ImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg);
  bool viewImage_;
  int waitKeyDelay_;
  bool droi_ready_ = false;
  bool isNodeRunning_ = true;
  bool controlDone_ = false;
  ros2_msg::msg::Lane2xav lane_coef_;
  int center_select_ = 1;

  // lane change
  bool L_flag = true;
  bool R_flag = true;

  //image
  bool imageStatus_ = false;
  cv::Mat camImageCopy_;
  cv::Mat cluster_frame;

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
  vector<Point2f> corners_, fROIcorners_;
  vector<Point2f> warpCorners_, fROIwarpCorners_;

  int last_Llane_base_;
  int last_Rlane_base_;

  vector<int> left_x_;
  vector<int> left_y_;
  vector<int> right_x_;
  vector<int> right_y_;
  vector<int> center_x_;
  vector<int> center_y_;

  int width_, height_;
  bool option_; // dynamic ROI
  int threshold_;

  int Threshold_box_size_, Threshold_box_offset_;

};

}
