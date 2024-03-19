#include "lane_detect.hpp"

namespace LaneDetect {

LaneDetector::LaneDetector()
       : Node("LaneDetector", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
  /**************/
  /* ROS2 Topic */
  /**************/
  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string ImageSubTopicName;
  int ImageSubQueueSize;
  std::string rearImageSubTopicName;
  int rearImageSubQueueSize;

  std::string XavPubTopicName;
  int XavPubQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("subscribers/xavier_to_lane/topic", XavSubTopicName, std::string("xav2lane_msg"));
  this->get_parameter_or("subscribers/xavier_to_lane/queue_size", XavSubQueueSize, 1);
  this->get_parameter_or("subscribers/image_to_lane/topic", ImageSubTopicName, std::string("usb_cam/image_raw"));
  this->get_parameter_or("subscribers/image_to_lane/queue_size", ImageSubQueueSize, 1);
  this->get_parameter_or("subscribers/rearImage_to_lane/topic", rearImageSubTopicName, std::string("rear_cam/image_raw"));
  this->get_parameter_or("subscribers/rearImage_to_lane/queue_size", rearImageSubQueueSize, 1);

  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  this->get_parameter_or("publishers/lane_to_xavier/topic", XavPubTopicName, std::string("lane2xav_msg"));
  this->get_parameter_or("publishers/lane_to_xavier/queue_size", XavPubQueueSize, 1);
  
  /************************/
  /* Ros Topic Subscriber */
  /************************/
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.best_effort();
  XavSubscriber_ = this->create_subscription<ros2_msg::msg::Xav2lane>(XavSubTopicName, XavSubQueueSize, std::bind(&LaneDetector::XavSubCallback, this, std::placeholders::_1));

  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(ImageSubTopicName, qos, std::bind(&LaneDetector::ImageSubCallback, this, std::placeholders::_1));

  rearImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(rearImageSubTopicName, rearImageSubQueueSize, std::bind(&LaneDetector::rearImageSubCallback, this, std::placeholders::_1));

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  XavPublisher_ = this->create_publisher<ros2_msg::msg::Lane2xav>(XavPubTopicName, XavPubQueueSize);
  SteerPublisher_ = this->create_publisher<std_msgs::msg::Float32>("steer",XavPubQueueSize);
  /***************/
  /* View Option */
  /***************/
  this->get_parameter_or("image_view/enable_opencv", viewImage_, true);
  this->get_parameter_or("image_view/wait_key_delay", waitKeyDelay_, 3);
  this->get_parameter_or("image_view/TEST", TEST, false);

  /******* recording log *******/    
  gettimeofday(&start_, NULL);

      /******* Camera  calibration *******/
  double f_matrix[9], f_dist_coef[5], r_matrix[9], r_dist_coef[5];
  this->get_parameter_or("Calibration/f_matrix/a",f_matrix[0], 3.2918100682757097e+02);
  this->get_parameter_or("Calibration/f_matrix/b",f_matrix[1], 0.);
  this->get_parameter_or("Calibration/f_matrix/c",f_matrix[2], 320.);
  this->get_parameter_or("Calibration/f_matrix/d",f_matrix[3], 0.);
  this->get_parameter_or("Calibration/f_matrix/e",f_matrix[4], 3.2918100682757097e+02);
  this->get_parameter_or("Calibration/f_matrix/f",f_matrix[5], 240.);
  this->get_parameter_or("Calibration/f_matrix/g",f_matrix[6], 0.);
  this->get_parameter_or("Calibration/f_matrix/h",f_matrix[7], 0.);
  this->get_parameter_or("Calibration/f_matrix/i",f_matrix[8], 1.);

  this->get_parameter_or("Calibration/f_dist_coef/a",f_dist_coef[0], -3.2566540239089398e-01);
  this->get_parameter_or("Calibration/f_dist_coef/b",f_dist_coef[1], 1.1504807178349362e-01);
  this->get_parameter_or("Calibration/f_dist_coef/c",f_dist_coef[2], 0.);
  this->get_parameter_or("Calibration/f_dist_coef/d",f_dist_coef[3], 0.);
  this->get_parameter_or("Calibration/f_dist_coef/e",f_dist_coef[4], -2.1908791800876997e-02);

  this->get_parameter_or("Calibration/r_matrix/a",r_matrix[0], 326.31389227574556);
  this->get_parameter_or("Calibration/r_matrix/b",r_matrix[1], 0.);
  this->get_parameter_or("Calibration/r_matrix/c",r_matrix[2], 320.);
  this->get_parameter_or("Calibration/r_matrix/d",r_matrix[3], 0.);
  this->get_parameter_or("Calibration/r_matrix/e",r_matrix[4], 326.31389227574556);
  this->get_parameter_or("Calibration/r_matrix/f",r_matrix[5], 240.);
  this->get_parameter_or("Calibration/r_matrix/g",r_matrix[6], 0.);
  this->get_parameter_or("Calibration/r_matrix/h",r_matrix[7], 0.);
  this->get_parameter_or("Calibration/r_matrix/i",r_matrix[8], 1.);

  this->get_parameter_or("Calibration/r_dist_coef/a",r_dist_coef[0], -0.33295846454356126);
  this->get_parameter_or("Calibration/r_dist_coef/b",r_dist_coef[1], 0.12386827336557986);
  this->get_parameter_or("Calibration/r_dist_coef/c",r_dist_coef[2], 0.);
  this->get_parameter_or("Calibration/r_dist_coef/d",r_dist_coef[3], 0.);
  this->get_parameter_or("Calibration/r_dist_coef/e",r_dist_coef[4], -0.022565312043601477);

  /*** front cam calibration  ***/
  f_camera_matrix = Mat::eye(3, 3, CV_64FC1);
  f_dist_coeffs = Mat::zeros(1, 5, CV_64FC1);
  f_camera_matrix = (Mat1d(3, 3) << f_matrix[0], f_matrix[1], f_matrix[2], f_matrix[3], f_matrix[4], f_matrix[5], f_matrix[6], f_matrix[7], f_matrix[8]);
  f_dist_coeffs = (Mat1d(1, 5) << f_dist_coef[0], f_dist_coef[1], f_dist_coef[2], f_dist_coef[3], f_dist_coef[4]);
  //initUndistortRectifyMap(f_camera_matrix, f_dist_coeffs, Mat(), f_camera_matrix, Size(640, 480), CV_32FC1, f_map1_, f_map2_);

  /*** rear cam calibration  ***/
  r_camera_matrix = Mat::eye(3, 3, CV_64FC1);
  r_dist_coeffs = Mat::zeros(1, 5, CV_64FC1);
  r_camera_matrix = (Mat1d(3, 3) << r_matrix[0], r_matrix[1], r_matrix[2], r_matrix[3], r_matrix[4], r_matrix[5], r_matrix[6], r_matrix[7], r_matrix[8]);
  r_dist_coeffs = (Mat1d(1, 5) << r_dist_coef[0], r_dist_coef[1], r_dist_coef[2], r_dist_coef[3], r_dist_coef[4]);
  //initUndistortRectifyMap(r_camera_matrix, r_dist_coeffs, Mat(), r_camera_matrix, Size(640, 480), CV_32FC1, r_map1_, r_map2_);

  //map1_ = f_map1_.clone();
  //map2_ = f_map2_.clone();
int width = 640;   // 이미지 너비
int height = 480;  // 이미지 높이

// map1_은 x좌표를 나타냄, 각 픽셀의 x좌표는 그 자체의 x좌표와 동일
map1_ = cv::Mat(height, width, CV_32FC1);
for(int y = 0; y < map1_.rows; y++) {
    for(int x = 0; x < map1_.cols; x++) {
        map1_.at<float>(y, x) = x;
    }
}

// map2_는 y좌표를 나타냄, 각 픽셀의 y좌표는 그 자체의 y좌표와 동일
map2_ = cv::Mat(height, width, CV_32FC1);
for(int y = 0; y < map2_.rows; y++) {
    for(int x = 0; x < map2_.cols; x++) {
        map2_.at<float>(y, x) = y;
    }
}
  /********** PID control ***********/
  prev_err_ = 0;

  last_Llane_base_ = 0;
  last_Rlane_base_ = 0;
  left_coef_ = Mat::zeros(3, 1, CV_32F);
  right_coef_ = Mat::zeros(3, 1, CV_32F);
  center_coef_ = Mat::zeros(3, 1, CV_32F);
  center2_coef_ = Mat::zeros(3, 1, CV_32F);
  center3_coef_ = Mat::zeros(3, 1, CV_32F);
  extra_coef_ = Mat::zeros(3, 1, CV_32F);
  extra2_coef_ = Mat::zeros(3, 1, CV_32F);

  this->get_parameter_or("ROI/width", width_, 640);
  this->get_parameter_or("ROI/height", height_, 480);
  center_position_ = width_/2;

  float t_gap[5], b_gap[5], t_height[5], b_height[5], f_extra[5], b_extra[5];
  int top_gap[5], bot_gap[5], top_height[5], bot_height[5], extra_up[5], extra_down[5];
  float wide_extra_upside_[5], wide_extra_downside_[5];

  this->get_parameter_or("ROI/dynamic_roi",option_, true);
  this->get_parameter_or("ROI/threshold",threshold_, 128);
  this->get_parameter_or("ROI/frontRoi_ratio",frontRoi_ratio, 204.0f);
  this->get_parameter_or("ROI/rearRoi_ratio",rearRoi_ratio, 210.0f);

  this->get_parameter_or("ROI/front_cam/top_gap",t_gap[0], 0.336f);
  this->get_parameter_or("ROI/front_cam/bot_gap",b_gap[0], 0.078f);
  this->get_parameter_or("ROI/front_cam/top_height",t_height[0], 0.903f);
  this->get_parameter_or("ROI/front_cam/bot_height",b_height[0], 0.528f);
  this->get_parameter_or("ROI/front_cam/extra_f",f_extra[0], 0.0f);
  this->get_parameter_or("ROI/front_cam/extra_b",b_extra[0], 0.0f);
  this->get_parameter_or("ROI/front_cam/extra_up",extra_up[0], 0);
  this->get_parameter_or("ROI/front_cam/extra_down",extra_down[0], 0);

  this->get_parameter_or("ROI/wide_right/top_gap",t_gap[1], 0.886f);
  this->get_parameter_or("ROI/wide_right/bot_gap",b_gap[1], 0.078f);
  this->get_parameter_or("ROI/wide_right/top_height",t_height[1], 0.903f);
  this->get_parameter_or("ROI/wide_right/bot_height",b_height[1], 0.528f);
  this->get_parameter_or("ROI/wide_right/extra_f",f_extra[1], 0.0f);
  this->get_parameter_or("ROI/wide_right/extra_b",b_extra[1], 0.0f);
  this->get_parameter_or("ROI/wide_right/extra_up",extra_up[1], 0);
  this->get_parameter_or("ROI/wide_right/extra_down",extra_down[1], 0);

  this->get_parameter_or("ROI/wide_left/top_gap",t_gap[2], 0.886f);
  this->get_parameter_or("ROI/wide_left/bot_gap",b_gap[2], 0.078f);
  this->get_parameter_or("ROI/wide_left/top_height",t_height[2], 0.903f);
  this->get_parameter_or("ROI/wide_left/bot_height",b_height[2], 0.528f);
  this->get_parameter_or("ROI/wide_left/extra_f",f_extra[2], 0.0f);
  this->get_parameter_or("ROI/wide_left/extra_b",b_extra[2], 0.0f);
  this->get_parameter_or("ROI/wide_left/extra_up",extra_up[2], 0);
  this->get_parameter_or("ROI/wide_left/extra_down",extra_down[2], 0);

  this->get_parameter_or("ROI/rear_cam/top_gap",t_gap[3], 0.405f);
  this->get_parameter_or("ROI/rear_cam/bot_gap",b_gap[3], 0.17f);
  this->get_parameter_or("ROI/rear_cam/top_height",t_height[3], 0.99f);
  this->get_parameter_or("ROI/rear_cam/bot_height",b_height[3], 0.47f);
  this->get_parameter_or("ROI/rear_cam/extra_f",f_extra[3], 1.0f);
  this->get_parameter_or("ROI/rear_cam/extra_b",b_extra[3], 10.0f);
  this->get_parameter_or("ROI/rear_cam/extra_up",extra_up[3], 140);
  this->get_parameter_or("ROI/rear_cam/extra_down",extra_down[3], 180);

  this->get_parameter_or("ROI/test/top_gap",t_gap[4], 0.405f);
  this->get_parameter_or("ROI/test/bot_gap",b_gap[4], 0.17f);
  this->get_parameter_or("ROI/test/top_height",t_height[4], 0.99f);
  this->get_parameter_or("ROI/test/bot_height",b_height[4], 0.47f);
  this->get_parameter_or("ROI/test/extra_f",f_extra[4], 1.0f);
  this->get_parameter_or("ROI/test/extra_b",b_extra[4], 10.0f);
  this->get_parameter_or("ROI/test/extra_up",extra_up[4], 140);
  this->get_parameter_or("ROI/test/extra_down",extra_down[4], 180);

  this->get_parameter_or("threshold/box_size", Threshold_box_size_, 51);
  this->get_parameter_or("threshold/box_offset", Threshold_box_offset_, 50);

  distance_ = 0;

  corners_.resize(4);
  warpCorners_.resize(4);

  test_corners_.resize(4);
  test_warpCorners_.resize(4);

  e_values_.resize(3);

  /*** front cam ROI setting ***/
  fROIcorners_.resize(4);
  fROIwarpCorners_.resize(4);

  top_gap[0] = width_ * t_gap[0]; 
  bot_gap[0] = width_ * b_gap[0];
  top_height[0] = height_ * t_height[0];
  bot_height[0] = height_ * b_height[0];

  fROIcorners_[0] = Point2f(top_gap[0]+f_extra[0], bot_height[0]);
  fROIcorners_[1] = Point2f((width_ - top_gap[0])+f_extra[0], bot_height[0]);
  fROIcorners_[2] = Point2f(bot_gap[0]+b_extra[0], top_height[0]);
  fROIcorners_[3] = Point2f((width_ - bot_gap[0])+b_extra[0], top_height[0]);
  
  wide_extra_upside_[0] = extra_up[0];
  wide_extra_downside_[0] = extra_down[0];
  
  fROIwarpCorners_[0] = Point2f(wide_extra_upside_[0], 0.0);
  fROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[0], 0.0);
  fROIwarpCorners_[2] = Point2f(wide_extra_downside_[0], height_);
  fROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[0], height_);
  /*** front cam ROI setting ***/

  /*** Wide Right ROI setting ***/
  rROIcorners_.resize(4);
  rROIwarpCorners_.resize(4);

  top_gap[1] = width_ * t_gap[1];
  bot_gap[1] = width_ * b_gap[1];
  top_height[1] = height_ * t_height[1];
  bot_height[1] = height_ * b_height[1];

  rROIcorners_[0] = Point2f(top_gap[1]+f_extra[1], bot_height[1]);
  rROIcorners_[1] = Point2f((width_ - top_gap[1])+f_extra[1], bot_height[1]);
  rROIcorners_[2] = Point2f(bot_gap[1]+b_extra[1], top_height[1]);
  rROIcorners_[3] = Point2f((width_ - bot_gap[1])+b_extra[1], top_height[1]);

  wide_extra_upside_[1] = extra_up[1];
  wide_extra_downside_[1] = extra_down[1];

  rROIwarpCorners_[0] = Point2f(wide_extra_upside_[1], 0.0);
  rROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[1], 0.0);
  rROIwarpCorners_[2] = Point2f(wide_extra_downside_[1], height_);
  rROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[1], height_);
  /*** Wide Right ROI setting ***/

  /*** Wide left ROI setting ***/
  lROIcorners_.resize(4);
  lROIwarpCorners_.resize(4);

  top_gap[2] = width_ * t_gap[2];
  bot_gap[2] = width_ * b_gap[2];
  top_height[2] = height_ * t_height[2];
  bot_height[2] = height_ * b_height[2];

  lROIcorners_[0] = Point2f(top_gap[2]+f_extra[2], bot_height[2]);
  lROIcorners_[1] = Point2f((width_ - top_gap[2])+f_extra[2], bot_height[2]);
  lROIcorners_[2] = Point2f(bot_gap[2]+b_extra[2], top_height[2]);
  lROIcorners_[3] = Point2f((width_ - bot_gap[2])+b_extra[2], top_height[2]);

  wide_extra_upside_[2] = extra_up[2];
  wide_extra_downside_[2] = extra_down[2];

  lROIwarpCorners_[0] = Point2f(wide_extra_upside_[2], 0.0);
  lROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[2], 0.0);
  lROIwarpCorners_[2] = Point2f(wide_extra_downside_[2], height_);
  lROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[2], height_);
  /*** Wide left ROI setting ***/

  /*** rear cam ROI setting ***/
  rearROIcorners_.resize(4);
  rearROIwarpCorners_.resize(4);

  top_gap[3] = width_ * t_gap[3]; 
  bot_gap[3] = width_ * b_gap[3];
  top_height[3] = height_ * t_height[3];
  bot_height[3] = height_ * b_height[3];

  rearROIcorners_[0] = Point2f(top_gap[3]+f_extra[3], bot_height[3]);
  rearROIcorners_[1] = Point2f((width_ - top_gap[3])+f_extra[3], bot_height[3]);
  rearROIcorners_[2] = Point2f(bot_gap[3]+b_extra[3], top_height[3]);
  rearROIcorners_[3] = Point2f((width_ - bot_gap[3])+b_extra[3], top_height[3]);
  
  wide_extra_upside_[3] = extra_up[3];
  wide_extra_downside_[3] = extra_down[3];
  
  rearROIwarpCorners_[0] = Point2f(wide_extra_upside_[3], 0.0);
  rearROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[3], 0.0);
  rearROIwarpCorners_[2] = Point2f(wide_extra_downside_[3], height_);
  rearROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[3], height_);
  /*** rear cam ROI setting ***/

  /*** front cam ROI 2.5m setting ***/
  testROIcorners_.resize(4);
  testROIwarpCorners_.resize(4);

  top_gap[4] = width_ * t_gap[4]; 
  bot_gap[4] = width_ * b_gap[4];
  top_height[4] = height_ * t_height[4];
  bot_height[4] = height_ * b_height[4];

  testROIcorners_[0] = Point2f(top_gap[4]+f_extra[4], bot_height[4]);
  testROIcorners_[1] = Point2f((width_ - top_gap[4])+f_extra[4], bot_height[4]);
  testROIcorners_[2] = Point2f(bot_gap[4]+b_extra[4], top_height[4]);
  testROIcorners_[3] = Point2f((width_ - bot_gap[4])+b_extra[4], top_height[4]);
  
  wide_extra_upside_[4] = extra_up[4];
  wide_extra_downside_[4] = extra_down[4];
  
  testROIwarpCorners_[0] = Point2f(wide_extra_upside_[4], 0.0);
  testROIwarpCorners_[1] = Point2f(width_ - wide_extra_upside_[4], 0.0);
  testROIwarpCorners_[2] = Point2f(wide_extra_downside_[4], height_);
  testROIwarpCorners_[3] = Point2f(width_ - wide_extra_downside_[4], height_);
  /*** front cam ROI 2.5m setting ***/

  /*  Synchronisation         */
  cam_new_frame_arrived = false;
  rear_cam_new_frame_arrived = false;

  /* Lateral Control coefficient */
  this->get_parameter_or("params/K", K_, 0.15f);
  this->get_parameter_or("params/K3", K3_, 0.15f);
  this->get_parameter_or("params/K4", K4_, 0.15f);

  this->get_parameter_or("params/a/a", a_[0], 0.);
  this->get_parameter_or("params/a/b", a_[1], -0.37169);
  this->get_parameter_or("params/a/c", a_[2], 1.2602);
  this->get_parameter_or("params/a/d", a_[3], -1.5161);
  this->get_parameter_or("params/a/e", a_[4], 0.70696);

  this->get_parameter_or("params/b/a", b_[0], 0.);
  this->get_parameter_or("params/b/b", b_[1], -1.7536);
  this->get_parameter_or("params/b/c", b_[2], 5.0931);
  this->get_parameter_or("params/b/d", b_[3], -4.9047);
  this->get_parameter_or("params/b/e", b_[4], 1.6722);

  this->get_parameter_or("params/a2/a", a2_[0], 0.5);
  this->get_parameter_or("params/a2/b", a2_[1], -0.95);
  this->get_parameter_or("params/a2/c", a2_[2], 0.52);

  this->get_parameter_or("params/b2/a", b2_[0], -0.0875);
  this->get_parameter_or("params/b2/b", b2_[1], 0.0575);
  this->get_parameter_or("params/b2/c", b2_[2], 0.17);
  LoadParams();

  isNodeRunning_ = true;
  lanedetect_Thread = std::thread(&LaneDetector::lanedetectInThread, this);

}

LaneDetector::~LaneDetector(void) 
{
  isNodeRunning_ = false;

  /*  Unblock the other thread to shutdown the programm smoothly  */
  cam_new_frame_arrived = true;
  rear_cam_new_frame_arrived = true;
  cam_condition_variable.notify_one();
  rear_cam_condition_variable.notify_one();

  ros2_msg::msg::Lane2xav xav;
  xav.coef = lane_coef_.coef;
  xav.cur_angle = AngleDegree_;
  xav.cur_angle2 = SteerAngle2_;
  xav.e_values = e_values_;

  XavPublisher_->publish(xav);
  lanedetect_Thread.join();

  clear_release();
  RCLCPP_INFO(this->get_logger(), "Stop.");
}

void LaneDetector::lanedetectInThread()
{
  double diff_time=0.0, CycleTime_=0.0;
  int cnt = 0;
  const auto wait_duration = std::chrono::milliseconds(2000);

  while(!imageStatus_ && !rearImageStatus_) {

    /*  Synchronize at startup  */
      unique_lock<mutex> lock(cam_mutex);
      if(cam_condition_variable.wait_for(lock, wait_duration, [this] { return (imageStatus_ && !rearImageStatus_); } )) {
        /*  Startup done! We are ready to start */
        RCLCPP_INFO(this->get_logger(), "First image arrived.\n");
        break;
      } else {
        /*  Timeout - Still waiting....         */
        RCLCPP_INFO(this->get_logger(), "Waiting for image.\n");
        if(!isNodeRunning_) {
          return;
        }
      }
  }

  ros2_msg::msg::Lane2xav xav;
  std_msgs::msg::Float32 steer_;
  while(!controlDone_ && rclcpp::ok()) 
  {
    struct timeval start_time, end_time, cur_time;
    gettimeofday(&start_time, NULL);

    if(imageStatus_ && droi_ready_) { /* use front_cam  */

      /*  Synchronize ImageSubCallback and this thread  */
      {
        unique_lock<mutex> lock(cam_mutex);
        cam_condition_variable.wait(lock, [this] { return cam_new_frame_arrived; } );
        cam_new_frame_arrived = false;
      }
      AngleDegree_ = display_img(camImageCopy_, waitKeyDelay_, viewImage_);
      droi_ready_ = false;
     
      xav.coef = lane_coef_.coef;
      xav.cur_angle = AngleDegree_;
      if (wroi_flag_) xav.cur_angle2 = SteerAngle2_;
      else xav.cur_angle2 = AngleDegree_;
      xav.wroi_flag = wroi_flag_;
      xav.center_select = center_select_;
      xav.e_values = e_values_;
      xav.k1 = K1_;
      xav.k2 = K2_;
      xav.lc_center_follow = lc_center_follow_;
      xav.est_dist = est_dist_;
      xav.est_vel = est_vel_;
      steer_.data = AngleDegree_;
//      gettimeofday(&cur_time, NULL);
//      xav.stamp_sec = cur_time.tv_sec;
//      xav.stamp_usec = cur_time.tv_usec;
      SteerPublisher_->publish(steer_);
      XavPublisher_->publish(xav);
    }
    else if(rearImageStatus_) { /* use rear_cam  */ 

      /*  Synchronize ImageSubCallback and this thread  */
      {
        unique_lock<mutex> lock(rear_cam_mutex);
        rear_cam_condition_variable.wait(lock, [this] { return rear_cam_new_frame_arrived; } );
        rear_cam_new_frame_arrived = false;
      }

      AngleDegree_ = display_img(rearCamImageCopy_, waitKeyDelay_, viewImage_);
     
      xav.coef = lane_coef_.coef;
      xav.center_select = center_select_;
      xav.est_dist = est_dist_;
      xav.est_vel = est_vel_;
      XavPublisher_->publish(xav);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if(!isNodeRunning_) {
      controlDone_ = true;
      rclcpp::shutdown();
    }

    gettimeofday(&end_time, NULL);
    diff_time += ((end_time.tv_sec - start_time.tv_sec) * 1000.0) + ((end_time.tv_usec - start_time.tv_usec) / 1000.0);
    cnt++;

    CycleTime_ = diff_time / (double)cnt;

    if (cnt > 3000){
            diff_time = 0.0;
            cnt = 0;
    }
//    RCLCPP_INFO(this->get_logger(), "CycleTime_: %.3f\n", CycleTime_);
  }
}

void LaneDetector::LoadParams(void)
{
  this->get_parameter_or("LaneDetector/eL_height",eL_height_, 1.0f);  
  this->get_parameter_or("LaneDetector/e1_height",e1_height_, 1.0f);  
  this->get_parameter_or("LaneDetector/trust_height",trust_height_, 1.0f);  
  this->get_parameter_or("LaneDetector/lp",lp_, 756.0f);  
  this->get_parameter_or("LaneDetector/steer_angle",SteerAngle_, 0.0f);
  this->get_parameter_or("LaneDetector/steer_angle2",SteerAngle2_, 0.0f);
}

cv::Point2f LaneDetector::transformPoint(const cv::Point& pt, const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    std::vector<cv::Point2f> srcPoints = { cv::Point2f(pt.x, pt.y) }; // 정수형 좌표를 부동소수점으로 변환
    std::vector<cv::Point2f> dstPoints;

    cv::undistortPoints(srcPoints, dstPoints, camera_matrix, dist_coeffs, cv::noArray(), camera_matrix);

    if (dstPoints.size() > 0) {
        return dstPoints[0];
    } else {
        return cv::Point2f(pt.x, pt.y);  // 변환에 실패한 경우 원래 좌표 반환
    }
}

void LaneDetector::XavSubCallback(const ros2_msg::msg::Xav2lane::SharedPtr msg)
{
  if(imageStatus_) {
    cur_vel_ = msg->cur_vel;
    distance_ = msg->cur_dist; // for ICRA
    droi_ready_ = true;
  
    get_steer_coef(cur_vel_);
  
    lc_right_flag = msg->lc_right_flag;
    lc_left_flag = msg->lc_left_flag;

    cv::Point2f topLeft = transformPoint(cv::Point(msg->x, msg->y), f_camera_matrix, f_dist_coeffs);
    cv::Point2f bottomRight = transformPoint(cv::Point(msg->x + msg->w, msg->y + msg->h), f_camera_matrix, f_dist_coeffs);

    name_ = msg->name;
    x_ = static_cast<int>(topLeft.x);
    y_ = static_cast<int>(topLeft.y);
    w_ = static_cast<int>(bottomRight.x - topLeft.x);
    h_ = static_cast<int>(bottomRight.y - topLeft.y);
  }

  if(rearImageStatus_) {
    cur_vel_ = msg->cur_vel;

    cv::Point2f topLeft = transformPoint(cv::Point(msg->rx, msg->ry),r_camera_matrix, r_dist_coeffs);
    cv::Point2f bottomRight = transformPoint(cv::Point(msg->rx + msg->rw, msg->ry + msg->rh), r_camera_matrix, r_dist_coeffs);

    r_name_ = msg->r_name;
    rx_ = static_cast<int>(topLeft.x);
    ry_ = static_cast<int>(topLeft.y);
    rw_ = static_cast<int>(bottomRight.x - topLeft.x);
    rh_ = static_cast<int>(bottomRight.y - topLeft.y);
  }
}

void LaneDetector::ImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  static cv::Mat prev_img;
//  static double diff_time;
//  static double CycleTime_;
//  static float cnt;
//  struct timeval end_time;

  cv_bridge::CvImagePtr cam_image;
  try{
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception : %s", e.what());
  }

  if(!cam_image->image.empty()) {
//    /* delay time */
//    gettimeofday(&end_time, NULL);
//    imageHeader_ = msg->header;
//    diff_time += ((end_time.tv_sec - imageHeader_.stamp.sec) * 1000.0) + ((end_time.tv_usec - imageHeader_.stamp.nanosec/1000.0) / 1000.0);
//     
//    cnt++;
//
//    CycleTime_ = diff_time / (double)cnt;
//    RCLCPP_INFO(this->get_logger(), "Cycle Time        : %3.3f ms\n", CycleTime_);
//
//    if (cnt > 3000){
//      diff_time = 0.0;
//      cnt = 0;
//    }
//    /* delay time */

    camImageCopy_ = cam_image->image.clone();
    prev_img = camImageCopy_;
    imageStatus_ = true;
    cam_new_frame_arrived = true;
    cam_condition_variable.notify_one();
  }
  else if(!prev_img.empty()) {
    camImageCopy_ = prev_img;
  }
}

void LaneDetector::rearImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  Mat frame_;
  cv_bridge::CvImagePtr rear_cam_image;
  try{
    rear_cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception : %s", e.what());
  }

  if(!rear_cam_image->image.empty()) {
    rearImageHeader_ = msg->header;
    rearCamImageCopy_ = rear_cam_image->image.clone();
    frame_ = camImageCopy_;
    rearImageStatus_ = true;

    rear_cam_new_frame_arrived = true;
    rear_cam_condition_variable.notify_one();
  }
}

int LaneDetector::arrMaxIdx(int hist[], int start, int end, int Max) {
  int max_index = -1;
  int max_val = 0;
  int min_pix = 30 * width_ / 1280;

  if (end > Max)
    end = Max;

  for (int i = start; i < end; i++) {
    if (max_val < hist[i]) {
      max_val = hist[i];
      max_index = i;
    }
  }
  if ((max_index == -1) || (hist[max_index] < (size_t)min_pix)) {
//    cout << "ERROR : hist range" << endl;
    return -1;
  }
  return max_index;
}

std::vector<int> LaneDetector::clusterHistogram(int* hist, int cluster_num) {
  struct Cluster {
    int centerIndex;
    int maxValue;
    int maxValueIndex;
  };    

  // Prepare data for k-means
  std::vector<cv::Point2f> points;
  for (int i = 0; i < width_; ++i) {
    for (int j = 0; j < hist[i]; ++j) {
      points.push_back(cv::Point2f(i, j));
    }
  }

  // K-means cluster
  cv::Mat labels, centers;
  try
  {
    cv::kmeans(points, cluster_num, labels,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0),
        3, cv::KMEANS_PP_CENTERS, centers);
  } catch (const cv::Exception& e) {
    std::cerr << "Exception caught: " << e.what() << std::endl;
    std::cerr << "The error occurred in cv::kmeans()" << std::endl;
    std::vector<int> result;
    for (int i = 0; i < cluster_num; i++) {
      result.push_back(-1);
    }
    return result;
  }

  // Get the representative index of each cluster
  std::vector<Cluster> clusters_info(cluster_num);
  for (int i = 0; i < cluster_num; ++i) {
    clusters_info[i] = {round(centers.at<float>(i, 0)), 0, -1};
  }

  // Calculate max value index for each cluster
  for (int i = 0; i < points.size(); ++i) {
    int label = labels.at<int>(i);
    int pixel_count = points[i].y;
    int index = points[i].x;
    if (pixel_count > clusters_info[label].maxValue) {
      clusters_info[label].maxValue = pixel_count;
      clusters_info[label].maxValueIndex = index;
      if(pixel_count <= 30) { // min_pixel in cluster
        clusters_info[label].maxValueIndex = -1;
      }
    }
  }    

  // Sort clusters by center index
  std::sort(clusters_info.begin(), clusters_info.end(), [](const Cluster& a, const Cluster& b) {
      return a.centerIndex < b.centerIndex;
      });

  std::vector<cv::Scalar> colors = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0)};  // BGR format

  // Plot cluster_frame and clusters
  cluster_frame = cv::Mat::zeros(480, 640, CV_8UC3);
  for (int i = 0; i < 640; ++i) {
    cv::line(cluster_frame, cv::Point(i, 480), cv::Point(i, 480 - hist[i]), cv::Scalar(255, 255, 255));
  }

  for (int i = 0; i < points.size(); ++i) {
    int label = labels.at<int>(i);
    for (int j = 0; j < cluster_num; ++j) {
      if (clusters_info[j].centerIndex == round(centers.at<float>(label, 0))) {
        cv::circle(cluster_frame, cv::Point(points[i].x, 480 - points[i].y), 2, colors[j], -1);
        break;
      }
    }
  }

  // Prepare result
  std::vector<int> result;
  for (const auto& cluster : clusters_info) {
    result.push_back(cluster.maxValueIndex);
  }

  //    if(viewImage_){
        namedWindow("Histogram Clusters");
        moveWindow("Histogram Clusters", 710, 700);
        cv::imshow("Histogram Clusters", cluster_frame);
        cv::waitKey(2);
  //    }

  return result;
}

Mat LaneDetector::polyfit(vector<int> x_val, vector<int> y_val) {
  Mat coef(3, 1, CV_32F);
  int i, j, k, n, N;
  N = (int)x_val.size();
  n = 2;
  double* x, * y;
  x = new double[N];
  y = new double[N];
  for (int q = 0; q < N; q++) {
    x[q] = (double)(x_val[q]);
    y[q] = (double)(y_val[q]);
  }
  double* X;
  X = new double[2 * n + 1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
  for (i = 0; i < (2 * n + 1); i++)
  {
    X[i] = 0;
    for (j = 0; j < N; j++)
      X[i] = X[i] + pow(x[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
  }
  double** B, * a;
  B = new double* [n + 1];
  for (int i = 0; i < (n + 1); i++)
    B[i] = new double[n + 2];
  a = new double[n + 1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
  for (i = 0; i <= n; i++)
    for (j = 0; j <= n; j++)
      B[i][j] = X[i + j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
  double* Y;
  Y = new double[n + 1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
  for (i = 0; i < (n + 1); i++)
  {
    Y[i] = 0;
    for (j = 0; j < N; j++)
      Y[i] = Y[i] + pow(x[j], i) * y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
  }
  for (i = 0; i <= n; i++)
    B[i][n + 1] = Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
  n = n + 1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations

  for (i = 0; i < n; i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
    for (k = i + 1; k < n; k++)
      if (B[i][i] < B[k][i])
        for (j = 0; j <= n; j++)
        {
          double temp = B[i][j];
          B[i][j] = B[k][j];
          B[k][j] = temp;
        }

  for (i = 0; i < (n - 1); i++)            //loop to perform the gauss elimination
    for (k = i + 1; k < n; k++)
    {
      double t = B[k][i] / B[i][i];
      for (j = 0; j <= n; j++)
        B[k][j] = B[k][j] - t * B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
    }
  for (i = n - 1; i >= 0; i--)                //back-substitution
  {                        //x is an array whose values correspond to the values of x,y,z..
    a[i] = B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
    for (j = 0; j < n; j++)
      if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
        a[i] = a[i] - B[i][j] * a[j];
    a[i] = a[i] / B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    coef.at<float>(i, 0) = (float)a[i];
  }

  delete[] x;
  delete[] y;
  delete[] X;
  delete[] Y;
  delete[] B;
  delete[] a;

  return coef;
}

Mat LaneDetector::detect_lines_sliding_window(Mat _frame, bool _view) {
  Mat frame, result;
  int width = _frame.cols;
  int height = _frame.rows;

  _frame.copyTo(frame);
  Mat nonZero;
  findNonZero(frame, nonZero);

  vector<int> good_left_inds;
  vector<int> good_right_inds;
  vector<int> good_extra_inds;
  vector<int> good_extra2_inds;

  int* hist = new int[width];

  for (int i = 0; i < width; i++) {
    hist[i] = 0;
  }

//  distance: 위 아래 군집 형성
//  for (int j = distance_; j < height; j++) { 
  for (int j = 0; j < height; j++) { // 0 ==> height/2
    for (int i = 0; i < width; i++) {
      if (frame.at <uchar>(j, i) == 255) {
        hist[i] += 1;
      }
    }
  } 

  cvtColor(frame, result, COLOR_GRAY2BGR);

  int mid_point = width / 2; // 320
  int n_windows = 9;
  int margin = 120 * width / 1280;
  int min_pix = 40 * width / 1280;
//  int min_pix = 100 * width / 1280;

  int window_width = margin * 2;  // 120
  int window_height;
  int distance;
  L_flag = true;
  R_flag = true;
  E_flag = true;
  E2_flag = true;
  if(!lc_right_flag_) E_flag = false; // extra right lane is only need for right lc flag 
  if(!lc_left_flag_) E2_flag = false; // extra left lane is only need for left lc flag 
  
  if (option_) {
    window_height = (height >= distance_) ? ((height-distance_) / n_windows) : (height / n_windows);  // defalut = 53
   
    distance = distance_;
  } else {
    distance = 0;
    window_height = height / n_windows;
  }
  
//  int E2lane_base = arrMaxIdx(hist, 0, 140, width);
  int Llane_base = arrMaxIdx(hist, 100, mid_point, width); // 140 ~ mid
  int Rlane_base = arrMaxIdx(hist, mid_point, width-100, width); // mid ~ w-140
//  int Elane_base = arrMaxIdx(hist, width-140, width, width);
  int E2lane_base = 0, Elane_base = 0;
  int cluster_num = 2;
  std::vector<int> maxIndices;
//
//  // 곡선에서 차선변경 완료될 경우 arrMaxIdx 고정 범위에서 벗어남
//  if(E_flag != true && E2_flag != true) {
//    maxIndices = clusterHistogram(hist, cluster_num);    
//    Llane_base = maxIndices[0];
//    Rlane_base = maxIndices[1];
//  }
  if (E_flag == true || E2_flag == true) {
    cluster_num = 3;
    maxIndices = clusterHistogram(hist, cluster_num);    
    
    // check for difference less than or equal to 60
    for (size_t i = 0; i < maxIndices.size(); ++i) {
        if(maxIndices[i] == -1) {
          cluster_num = 2;
        }   
        for (size_t j = i+1; j < maxIndices.size(); ++j) {
          if (maxIndices[i] != -1 && maxIndices[j] != -1) {
            if (std::abs(maxIndices[i] - maxIndices[j]) <= 60) {
                cluster_num = 2;
                break;
            }
          }
        }
    }
    if (cluster_num == 2) {
        maxIndices = clusterHistogram(hist, cluster_num);    
    }
  
    if (cluster_num == 3) {
      if (E_flag == true) {
        Llane_base = maxIndices[0];
        Rlane_base = maxIndices[1];
        Elane_base = maxIndices[2];
      }
      else if (E2_flag == true) {
        E2lane_base = maxIndices[0];
        Llane_base = maxIndices[1];
        Rlane_base = maxIndices[2];
      }
    } 
    else if (cluster_num == 2) {
      if (E_flag == true) {
        Llane_base = -1;
        Rlane_base = maxIndices[0];
        Elane_base = maxIndices[1];
      }
      else if (E2_flag == true) {
        E2lane_base = maxIndices[0];
        Llane_base = maxIndices[1];
        Rlane_base = -1;
      }
    }
  }

  int Llane_current = Llane_base;
  int Rlane_current = Rlane_base;
  int Elane_current = Elane_base;
  int E2lane_current = E2lane_base;

  int L_prev =  Llane_current;
  int R_prev =  Rlane_current;
  int E_prev =  Elane_current;
  int E2_prev =  E2lane_current;
  int L_gap = 0;
  int R_gap = 0;
  int E_gap = 0;
  int E2_gap = 0;
  static int prev_L_gap = 0;
  static int prev_R_gap = 0;
  static int prev_E_gap = 0;
  static int prev_E2_gap = 0;
  static int prev_Llane_current = 0;
  static int prev_Rlane_current = 0;
  static int prev_Elane_current = 0;
  static int prev_E2lane_current = 0;

  unsigned int index;

  if (Llane_base == -1) {
  //  RCLCPP_ERROR(this->get_logger(), "Not Detection Llane_Base");
    L_flag = false;
    prev_Llane_current = 0;
    prev_L_gap = 0;
  }
  if (Rlane_base == -1) {
 //   RCLCPP_ERROR(this->get_logger(), "Not Detection Rlane_Base");
    R_flag = false;
    prev_Rlane_current = 0;
    prev_R_gap = 0;
  }
  if (Elane_base == -1) {
//    RCLCPP_INFO(this->get_logger(), "Not Detection Elane_Base");
    E_flag = false;
    prev_Elane_current = 0;
    prev_E_gap = 0;
  } 
  if (E2lane_base == -1) {
//    RCLCPP_INFO(this->get_logger(), "Not Detection E2lane_Base");
    E2_flag = false;
    prev_E2lane_current = 0;
    prev_E2_gap = 0;
  } 
//  RCLCPP_INFO(this->get_logger(), "E2lane | Llane | Rlane | Elane | E2_flag | E_flag : %d | %d | %d | %d | %d | %d \n", E2lane_base, Llane_base, Rlane_base, Elane_base, E2_flag, E_flag);

  for (int window = 0; window < n_windows; window++) {
    int  Ly_pos = height - (window + 1) * window_height - 1; // win_y_low , win_y_high = win_y_low - window_height
    int  Ry_pos = height - (window + 1) * window_height - 1;
    int  Ey_pos = height - (window + 1) * window_height - 1;
    int  E2y_pos = height - (window + 1) * window_height - 1;
    int  Ly_top = height - window * window_height;
    int  Ry_top = height - window * window_height;
    int  Ey_top = height - window * window_height;
    int  E2y_top = height - window * window_height;

    int  Lx_pos = Llane_current - margin; // win_xleft_low, win_xleft_high = win_xleft_low + margin*2
    int  Rx_pos = Rlane_current - margin; // win_xrignt_low, win_xright_high = win_xright_low + margin*2
    int  Ex_pos = Elane_current - margin; // win_xrignt_low, win_xright_high = win_xright_low + margin*2
    int  E2x_pos = E2lane_current - margin; // win_xrignt_low, win_xright_high = win_xright_low + margin*2
    if (_view) {
      if(L_flag){
        rectangle(result, \
          Rect(Lx_pos, Ly_pos, window_width, window_height), \
          Scalar(255, 50, 100), 1);
      }
      if(R_flag){
        rectangle(result, \
          Rect(Rx_pos, Ry_pos, window_width, window_height), \
          //Scalar(100, 50, 255), 1);
          Scalar(255, 50, 100), 1);
      }
      if(E_flag){
        rectangle(result, \
          Rect(Ex_pos, Ey_pos, window_width, window_height), \
          //Scalar(50, 255, 255), 1);
          Scalar(255, 50, 100), 1);
      }
      if(E2_flag){
        rectangle(result, \
          Rect(E2x_pos, E2y_pos, window_width, window_height), \
          //Scalar(50, 255, 255), 1);
          Scalar(255, 50, 100), 1);
      }
    }
    int nZ_y, nZ_x;
    good_left_inds.clear();
    good_right_inds.clear();
    good_extra_inds.clear();
    good_extra2_inds.clear();

    for (int index = static_cast<int>(nonZero.total() - 1); index >= 0; index--) {
      nZ_y = nonZero.at<Point>(index).y;
      nZ_x = nonZero.at<Point>(index).x;
      if(L_flag){
        if ((nZ_y >= Ly_pos) && \
          (nZ_y > (distance)) && \
          (nZ_y < Ly_top) && \
          (nZ_x >= Lx_pos) && \
          (nZ_x < (Lx_pos + window_width))) {
          if (_view) {
            result.at<Vec3b>(nonZero.at<Point>(index))[0] = 255;
            result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
            result.at<Vec3b>(nonZero.at<Point>(index))[2] = 0;
          }
          good_left_inds.push_back(index);
        }
      }
      
      if(R_flag){
        if ((nZ_y >= (Ry_pos)) && \
          (nZ_y > (distance)) && \
          (nZ_y < Ry_top) && \
          (nZ_x >= Rx_pos) && \
          (nZ_x < (Rx_pos + window_width))) {
          if (_view) {
            result.at<Vec3b>(nonZero.at<Point>(index))[0] = 255;
            result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
            result.at<Vec3b>(nonZero.at<Point>(index))[2] = 0;
   //         result.at<Vec3b>(nonZero.at<Point>(index))[0] = 0;
   //         result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
   //         result.at<Vec3b>(nonZero.at<Point>(index))[2] = 255;
          }
          good_right_inds.push_back(index);
        }
      }

      if(E_flag){
        if ((nZ_y >= (Ey_pos)) && \
          (nZ_y > (distance)) && \
          (nZ_y < Ey_top) && \
          (nZ_x >= Ex_pos) && \
          (nZ_x < (Ex_pos + window_width))) {
          if (_view) {
            result.at<Vec3b>(nonZero.at<Point>(index))[0] = 255;
            result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
            result.at<Vec3b>(nonZero.at<Point>(index))[2] = 0;
//            result.at<Vec3b>(nonZero.at<Point>(index))[0] = 0;
//            result.at<Vec3b>(nonZero.at<Point>(index))[1] = 255;
//            result.at<Vec3b>(nonZero.at<Point>(index))[2] = 255;
          }
          good_extra_inds.push_back(index);
        }
      }
      if(E2_flag){
        if ((nZ_y >= (E2y_pos)) && \
          (nZ_y > (distance)) && \
          (nZ_y < E2y_top) && \
          (nZ_x >= E2x_pos) && \
          (nZ_x < (E2x_pos + window_width))) {
          if (_view) {
            result.at<Vec3b>(nonZero.at<Point>(index))[0] = 255;
            result.at<Vec3b>(nonZero.at<Point>(index))[1] = 0;
            result.at<Vec3b>(nonZero.at<Point>(index))[2] = 0;
//            result.at<Vec3b>(nonZero.at<Point>(index))[0] = 0;
//            result.at<Vec3b>(nonZero.at<Point>(index))[1] = 255;
//            result.at<Vec3b>(nonZero.at<Point>(index))[2] = 255;
          }
          good_extra2_inds.push_back(index);
        }
      }
    }
    
    int Lsum, Rsum, Esum, E2sum;
    Lsum = Rsum = Esum = E2sum = 0;
    unsigned int _size;
    vector<int> Llane_x;
    vector<int> Llane_y;
    vector<int> Rlane_x;
    vector<int> Rlane_y;
    vector<int> Elane_x;
    vector<int> Elane_y;
    vector<int> E2lane_x;
    vector<int> E2lane_y;


    if(L_flag) {
      if (good_left_inds.size() > (size_t)min_pix) {
        _size = (unsigned int)(good_left_inds.size());
        for (int i = Ly_top-1; i >= Ly_pos ; i--)
        {
          int Ly_sum = 0;
          int count = 0;
          for (index = 0; index < _size; index++) {
            int j = nonZero.at<Point>(good_left_inds.at(index)).y;
            if(i == j)
            {
              Ly_sum += nonZero.at<Point>(good_left_inds.at(index)).x;
              count++;
              Lsum += nonZero.at<Point>(good_left_inds.at(index)).x;
            }
          }
          if(count != 0)
          {
            left_x_.insert(left_x_.end(), Ly_sum/count);
            left_y_.insert(left_y_.end(), i);
            Llane_x.insert(Llane_x.end(), Ly_sum/count);
            Llane_y.insert(Llane_y.end(), i);
          } else {
            Llane_x.insert(Llane_x.end(), -1);
            Llane_y.insert(Llane_y.end(), i);
          }
        }
        Llane_current = Lsum / _size;
  if(window == 0) {
          prev_Llane_current = Llane_base;
  }
      }
      else {
//        if (window == 0 && prev_Llane_current != 0) {
//          Llane_current = prev_Llane_current;
//        } else if (window == 1 && prev_L_gap != 0) {
//          Llane_current += prev_L_gap;
//        } else {
//          Llane_current += (L_gap);
//  }
        Llane_current += (L_gap);
      }
    }

    if(R_flag) {
      if (good_right_inds.size() > (size_t)min_pix) {
        _size = (unsigned int)(good_right_inds.size());
        for (int i = Ry_top - 1 ; i >= Ry_pos ; i--)
        {
          int Ry_sum = 0;
          int count = 0;
          for (index = 0; index < _size; index++) {
            int j = nonZero.at<Point>(good_right_inds.at(index)).y;
            if(i == j)
            {
              Ry_sum += nonZero.at<Point>(good_right_inds.at(index)).x;
              count++;
              Rsum += nonZero.at<Point>(good_right_inds.at(index)).x;
            }
          }
          if(count != 0)
          {
            right_x_.insert(right_x_.end(), Ry_sum/count);
            right_y_.insert(right_y_.end(), i);
            Rlane_x.insert(Rlane_x.end(), Ry_sum/count);
            Rlane_y.insert(Rlane_y.end(), i);
          } else {
            Rlane_x.insert(Rlane_x.end(), -1);
            Rlane_y.insert(Rlane_y.end(), i);
          }
        }
        Rlane_current = Rsum / _size;
  if(window == 0) {
          prev_Rlane_current = Rlane_base;
  }
      }
      else {
//        if (window == 0 && prev_Rlane_current != 0) {
//          Rlane_current = prev_Rlane_current;
//        } else if (window == 1 && prev_R_gap != 0) {
//          Rlane_current += prev_R_gap;
//        } else {
//          Rlane_current += (R_gap);
//  }
        Rlane_current += (R_gap);
      }
    }


    if(E_flag) {
      if ((good_extra_inds.size() > (size_t)min_pix)) {
        _size = (unsigned int)(good_extra_inds.size());
        for (int i = Ey_top - 1 ; i >= Ey_pos ; i--)
        {
          int Ey_sum = 0;
          int count = 0;
          for (index = 0; index < _size; index++) {
            int j = nonZero.at<Point>(good_extra_inds.at(index)).y;
            if(i == j)
            {
              Ey_sum += nonZero.at<Point>(good_extra_inds.at(index)).x;
              count++;
              Esum += nonZero.at<Point>(good_extra_inds.at(index)).x;
            }
          }
          if(count != 0)
          {
            extra_x_.insert(extra_x_.end(), Ey_sum/count);
            extra_y_.insert(extra_y_.end(), i);
            Elane_x.insert(Elane_x.end(), Ey_sum/count);
            Elane_y.insert(Elane_y.end(), i);
          } else {
            Elane_x.insert(Elane_x.end(), -1);
            Elane_y.insert(Elane_y.end(), i);
          }
        }
        Elane_current = Esum / _size;
  if(window == 0) {
          prev_Elane_current = Elane_base;
  }
      } 
      else {
//        if (window == 0 && prev_Elane_current != 0) {
//          Elane_current = prev_Elane_current;
//        } else if (window == 1 && prev_E_gap != 0) {
//          Elane_current += prev_E_gap;
//        } else {
//          Elane_current += (E_gap);
//  }
        Elane_current += (E_gap);
      }
    }

    if (E2_flag) {
      if ((good_extra2_inds.size() > (size_t)min_pix)) {
        _size = (unsigned int)(good_extra2_inds.size());
        for (int i = E2y_top - 1 ; i >= E2y_pos ; i--)
        {
          int E2y_sum = 0;
          int count = 0;
          for (index = 0; index < _size; index++) {
            int j = nonZero.at<Point>(good_extra2_inds.at(index)).y;
            if(i == j)
            {
              E2y_sum += nonZero.at<Point>(good_extra2_inds.at(index)).x;
              count++;
              E2sum += nonZero.at<Point>(good_extra2_inds.at(index)).x;
            }
          }
          if(count != 0)
          {
            extra2_x_.insert(extra2_x_.end(), E2y_sum/count);
            extra2_y_.insert(extra2_y_.end(), i);
            E2lane_x.insert(E2lane_x.end(), E2y_sum/count);
            E2lane_y.insert(E2lane_y.end(), i);
          } else {
            E2lane_x.insert(E2lane_x.end(), -1);
            E2lane_y.insert(E2lane_y.end(), i);
          }
        }
        E2lane_current = E2sum / _size;
  if(window == 0) {
          prev_E2lane_current = E2lane_base;
  }
      } 
      else {
//        if (window == 0 && prev_E2lane_current != 0) {
//          E2lane_current = prev_E2lane_current;
//        } else if (window == 1 && prev_E2_gap != 0) {
//          E2lane_current += prev_E2_gap;
//        } else {
//          E2lane_current += (E2_gap);
//  }
        E2lane_current += (E2_gap);
      }
    }

    if (window != 0) {  
      if (Rlane_current != R_prev) {
        R_gap = (Rlane_current - R_prev);
  if(window == 1) {
    prev_R_gap = R_gap;
  }
      }
      if (Llane_current != L_prev) {
        L_gap = (Llane_current - L_prev);
  if(window == 1) {
    prev_L_gap = L_gap;
  }
      }
      if((Elane_current != E_prev) && E_flag) {
        E_gap = (Elane_current - E_prev);
  if(window == 1) {
    prev_E_gap = E_gap;
  }
      }
      if ((E2lane_current != E2_prev) && E2_flag) {
        E2_gap = (E2lane_current - E2_prev);
  if(window == 1) {
    prev_E2_gap = E2_gap;
  }
      }
    }
    /* center1  */
    if ((Lsum != 0) && (Rsum != 0)) {
	    int c_cnt = 0;
	    int l_cnt = 0;
	    int r_cnt = 0;
      for (int i = 0; i < Llane_x.size() ; i++)
      {
        if((Llane_x.at(i) != -1) && (Rlane_x.at(i) != -1)) {
          center_x_.insert(center_x_.end(), (Llane_x.at(i)+Rlane_x.at(i)) / 2 );
          center_y_.insert(center_y_.end(), Llane_y.at(i));
        } 
	

      }
    }
    /* center2  */
    if ((Rsum != 0) && (Esum != 0)) {
      for (int i = 0; i < Rlane_x.size() ; i++)
      {
        if((Rlane_x.at(i) != -1) && (Elane_x.at(i) != -1)) {
          center2_x_.insert(center2_x_.end(), (Rlane_x.at(i)+Elane_x.at(i)) / 2 );
          center2_y_.insert(center2_y_.end(), Rlane_y.at(i));
        }
      }
    } 
    /* center3  */
    if ((E2sum != 0) && (Lsum != 0)) {
      for (int i = 0; i < E2lane_x.size() ; i++)
      {
        if((E2lane_x.at(i) != -1) && (Llane_x.at(i) != -1)) {
          center3_x_.insert(center3_x_.end(), (E2lane_x.at(i)+Llane_x.at(i)) / 2 );
          center3_y_.insert(center3_y_.end(), E2lane_y.at(i));
        }
      }
    } 

    L_prev = Llane_current;
    R_prev = Rlane_current;
    E_prev = Elane_current;
    E2_prev = E2lane_current;
  }
  
  if (left_x_.size() != 0) {
    left_coef_ = polyfit(left_y_, left_x_);
  }
  if (right_x_.size() != 0) {
    right_coef_ = polyfit(right_y_, right_x_);
  }
  if (extra_x_.size() != 0) {
    extra_coef_ = polyfit(extra_y_, extra_x_);
  }
  if (extra2_x_.size() != 0) {
    extra2_coef_ = polyfit(extra2_y_, extra2_x_);
  }
  
  int center_diff_ = 999, center2_diff_ = 999, center3_diff_ = 999;
  //if (center_x_.size() != 0){
  if (left_x_.size() != 0 && right_x_.size() != 0){ // && !center_x_.empty() remove
    //center_coef_ = polyfit(center_y_, center_x_);
    center_coef_ = (left_coef_ + right_coef_)/2;
    center_diff_ = abs(mid_point ); // - center_x_.front() remove
  }
 
  if (extra_x_.size() != 0 && right_x_.size() != 0 && !center2_x_.empty()){
//    center2_coef_ = polyfit(center2_y_, center2_x_);
    center2_coef_ = (right_coef_ + extra_coef_)/2;
    center2_diff_ = abs(mid_point - center2_x_.front());
  }
  if (left_x_.size() != 0 && extra2_x_.size() != 0 && !center3_x_.empty()){
//    center3_coef_ = polyfit(center3_y_, center3_x_);
    center3_coef_ = (extra2_coef_ + left_coef_)/2;
    center3_diff_ = abs(mid_point - center3_x_.front());
  }

//  printf("center | center2 : %d|%d\n", center_x_.front(), center2_x_.front());

  if ((center_diff_!= 999 && center2_diff_!= 999) || (center_diff_!= 999 && center3_diff_!= 999)) {
    if (center_diff_ < center2_diff_ && center_diff_ < center3_diff_) {
      center_select_ = 1;
    } else if (center2_diff_ < center_diff_ && center2_diff_ < center3_diff_) {
      center_select_ = 2;
    } else if (center3_diff_ < center_diff_ && center3_diff_ < center2_diff_) {
      center_select_ = 3;
    }
  }

  delete[] hist;

  return result;
}


float LaneDetector::lowPassFilter(double sampling_time, float est_value, float prev_res){
  float res = 0;
  float tau = 0.1f; 
  double st = 0.0;

  if (sampling_time > 1.0) st = 1.0;
  else st = sampling_time;
  res = ((tau * prev_res) + (st * est_value)) / (tau + st);

  return res;
}

float LaneDetector::lowPassFilter2(double sampling_time, float est_value, float prev_res){
  float res = 0;
  float tau = 0.5f; 
  double st = 0.0;

  if (sampling_time > 1.0) st = 1.0;
  else st = sampling_time;
  res = ((tau * prev_res) + (st * est_value)) / (tau + st);

  return res;
}

Point LaneDetector::warpPoint(Point center, Mat trans){
  Point warp_center, avg_center;

  warp_center.x = (trans.at<double>(0,0)*center.x + trans.at<double>(0,1)*center.y + trans.at<double>(0,2)) / (trans.at<double>(2,0)*center.x + trans.at<double>(2,1)*center.y + trans.at<double>(2,2));
  warp_center.y = (trans.at<double>(1,0)*center.x + trans.at<double>(1,1)*center.y + trans.at<double>(1,2)) / (trans.at<double>(2,0)*center.x + trans.at<double>(2,1)*center.y + trans.at<double>(2,2));

  return warp_center;
}

Mat LaneDetector::draw_lane(Mat _sliding_frame, Mat _frame) {
  Mat new_frame, left_coef(left_coef_), right_coef(right_coef_), extra_coef(extra_coef_), extra2_coef(extra2_coef_), center_coef(center_coef_), center2_coef(center2_coef_), center3_coef(center3_coef_), trans;

  static struct timeval endTime, startTime;
  static bool flag;
  double diffTime;

  trans = getPerspectiveTransform(fROIwarpCorners_, fROIcorners_);
 // if (imageStatus_ && TEST) {
 //   trans = getPerspectiveTransform(test_warpCorners_, test_corners_);
 // }
 // else {
 //   trans = getPerspectiveTransform(warpCorners_, corners_);
 // }
  _frame.copyTo(new_frame);

  vector<Point> left_point;
  vector<Point> right_point;
  vector<Point> extra_point;
  vector<Point> extra2_point;
  vector<Point> center_point;
  vector<Point> center2_point;
  vector<Point> center3_point;
  vector<Point> lc_point;
  vector<Point> lc2_point;

  vector<Point2f> left_point_f;
  vector<Point2f> right_point_f;
  vector<Point2f> extra_point_f;
  vector<Point2f> extra2_point_f;
  vector<Point2f> center_point_f;
  vector<Point2f> center2_point_f;
  vector<Point2f> center3_point_f;
  vector<Point2f> lc_point_f;
  vector<Point2f> lc2_point_f;

  vector<Point2f> warped_left_point;
  vector<Point2f> warped_right_point;
  vector<Point2f> warped_extra_point;
  vector<Point2f> warped_extra2_point;
  vector<Point2f> warped_center_point;
  vector<Point2f> warped_center2_point;
  vector<Point2f> warped_center3_point;
  vector<Point2f> warped_lc_point;
  vector<Point2f> warped_lc2_point;

  vector<Point> left_points;
  vector<Point> right_points;
  vector<Point> extra_points;
  vector<Point> extra2_points;
  vector<Point> center_points;
  vector<Point> center2_points;
  vector<Point> center3_points;
  vector<Point> lc_points;
  vector<Point> lc2_points;

  if ((!left_coef.empty()) && (!right_coef.empty())) {
    for (int i = 0; i <= height_; i++) {
      Point temp_left_point;
      Point temp_right_point;
      Point temp_center_point;

      temp_left_point.x = (int)((left_coef.at<float>(2, 0) * pow(i, 2)) + (left_coef.at<float>(1, 0) * i) + left_coef.at<float>(0, 0));
      temp_left_point.y = (int)i;
      temp_right_point.x = (int)((right_coef.at<float>(2, 0) * pow(i, 2)) + (right_coef.at<float>(1, 0) * i) + right_coef.at<float>(0, 0));
      temp_right_point.y = (int)i;
      temp_center_point.x = (int)((center_coef.at<float>(2, 0) * pow(i, 2)) + (center_coef.at<float>(1, 0) * i) + center_coef.at<float>(0, 0));
      temp_center_point.y = (int)i;

      left_point.push_back(temp_left_point);
      left_point_f.push_back(temp_left_point);
      right_point.push_back(temp_right_point);
      right_point_f.push_back(temp_right_point);
      center_point.push_back(temp_center_point);
      center_point_f.push_back(temp_center_point);
    }
    const Point* left_points_point_ = (const cv::Point*) Mat(left_point).data;
    int left_points_number_ = Mat(left_point).rows;
    const Point* right_points_point_ = (const cv::Point*) Mat(right_point).data;
    int right_points_number_ = Mat(right_point).rows;
    const Point* center_points_point_ = (const cv::Point*) Mat(center_point).data;
    int center_points_number_ = Mat(center_point).rows;

    if(L_flag == true){
      polylines(_sliding_frame, &left_points_point_, &left_points_number_, 1, false, Scalar(255, 100, 100), 10);
    } 
    if(R_flag == true){
      polylines(_sliding_frame, &right_points_point_, &right_points_number_, 1, false, Scalar(255, 100, 100), 10);
    }
    if(L_flag == true && R_flag == true){
      polylines(_sliding_frame, &center_points_point_, &center_points_number_, 1, false, Scalar(200, 255, 200), 10);
    }
    
    perspectiveTransform(left_point_f, warped_left_point, trans);
    perspectiveTransform(right_point_f, warped_right_point, trans);
    perspectiveTransform(center_point_f, warped_center_point, trans);

    for (int i = 0; i <= height_; i++) {
      Point temp_left_point;
      Point temp_right_point;
      Point temp_center_point;

      temp_left_point.x = (int)warped_left_point[i].x;
      temp_left_point.y = (int)warped_left_point[i].y;
      temp_right_point.x = (int)warped_right_point[i].x;
      temp_right_point.y = (int)warped_right_point[i].y;
      temp_center_point.x = (int)warped_center_point[i].x;
      temp_center_point.y = (int)warped_center_point[i].y;

      left_points.push_back(temp_left_point);
      right_points.push_back(temp_right_point);
      center_points.push_back(temp_center_point);
    }

    const Point* left_points_point = (const cv::Point*) Mat(left_points).data;
    int left_points_number = Mat(left_points).rows;
    const Point* right_points_point = (const cv::Point*) Mat(right_points).data;
    int right_points_number = Mat(right_points).rows;
    const Point* center_points_point = (const cv::Point*) Mat(center_points).data;
    int center_points_number = Mat(center_points).rows;

    Point lane_center = *(center_points_point + center_points_number - 10);

    static Point prev_lane_center;

    gettimeofday(&endTime, NULL);
    if (!flag){
      diffTime = (endTime.tv_sec - start_.tv_sec) + (endTime.tv_usec - start_.tv_usec)/1000000.0;
      flag = true;
    }
    else{
      diffTime = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec)/1000000.0;
      startTime = endTime;
    }
    lane_center.x = lowPassFilter(diffTime, lane_center.x, prev_lane_center.x);
    lane_center.y = lowPassFilter(diffTime, lane_center.y, prev_lane_center.y);

    prev_lane_center = lane_center;
    
    if(L_flag == true) 
      polylines(new_frame, &left_points_point, &left_points_number, 1, false, Scalar(255, 100, 100), 10);
    if(R_flag == true && !E2_flag)
      //polylines(new_frame, &right_points_point, &right_points_number, 1, false, Scalar(100, 100, 255), 5);
      polylines(new_frame, &right_points_point, &right_points_number, 1, false, Scalar(255, 100, 100), 10);
    if(L_flag == true && R_flag == true && !E2_flag)
      polylines(new_frame, &center_points_point, &center_points_number, 1, false, Scalar(100, 255, 100), 10);
    
    left_point.clear();
    right_point.clear();
    center_point.clear();

    /***************/
    /* Dynamic ROI */
    /***************/

    Point temp_roi_point;
    Point temp_droi_point;
    vector<Point2f> droi_point_f;
    vector<Point2f> warped_droi_point;
    vector<Point> roi_points;
    vector<Point> droi_points;
    
    temp_droi_point.y = (int)height_;
    temp_droi_point.x = 0;
    droi_point_f.push_back(temp_droi_point); //droi[0]
    temp_droi_point.x = (int)width_;
    droi_point_f.push_back(temp_droi_point); //droi[1]
    
    temp_droi_point.y = distance_;
    temp_droi_point.x = (int)width_;
    droi_point_f.push_back(temp_droi_point); //droi[2]
    temp_droi_point.x = 0;
    droi_point_f.push_back(temp_droi_point); //droi[3]
     
    perspectiveTransform(droi_point_f, warped_droi_point, trans);
    
    int droi_num[5] = {0, 1, 2, 3, 0};
    int roi_num[5] = {0, 1, 3, 2, 0};
    
    for (int i = 0; i < 5; i++) {
      temp_droi_point.x = (int)warped_droi_point[droi_num[i]].x;
      temp_droi_point.y = (int)warped_droi_point[droi_num[i]].y;
      
      droi_points.push_back(temp_droi_point);
      
      if(imageStatus_ && TEST) {
        temp_roi_point.x = (int)test_corners_[roi_num[i]].x;
        temp_roi_point.y = (int)test_corners_[roi_num[i]].y;
      }
      else {
        temp_roi_point.x = (int)corners_[roi_num[i]].x;
        temp_roi_point.y = (int)corners_[roi_num[i]].y;
      }
      roi_points.push_back(temp_roi_point);
    }

    const Point* roi_points_point = (const cv::Point*) Mat(roi_points).data;
    int roi_points_number = Mat(roi_points).rows;
    const Point* droi_points_point = (const cv::Point*) Mat(droi_points).data;
    int droi_points_number = Mat(droi_points).rows;

    polylines(_frame, &roi_points_point, &roi_points_number, 1, false, Scalar(0, 0, 255), 5);
    polylines(_frame, &droi_points_point, &droi_points_number, 1, false, Scalar(0, 255, 0), 5);

    string TEXT = "ROI";
    Point2f T_pos(Point2f(270, _frame.rows-120));
    putText(_frame, TEXT, T_pos, FONT_HERSHEY_DUPLEX, 2, Scalar(0, 255, 0), 5, 8);

//    return new_frame;
  }

  if ((!right_coef.empty()) && (!extra_coef.empty()) && E_flag == true) {
    mark_ = 1;
    tk::spline cspline_eq_ = cspline(); // s : lane2 coef

    for (int i = 0; i <= height_; i++) {
      Point temp_extra_point;
      Point temp_center2_point;
      Point temp_lc_point;

      temp_extra_point.x = (int)((extra_coef.at<float>(2, 0) * pow(i, 2)) + (extra_coef.at<float>(1, 0) * i) + extra_coef.at<float>(0, 0));
      temp_extra_point.y = (int)i;
      temp_center2_point.x = (int)((center2_coef.at<float>(2, 0) * pow(i, 2)) + (center2_coef.at<float>(1, 0) * i) + center2_coef.at<float>(0, 0));
      temp_center2_point.y = (int)i;
      temp_lc_point.x = (int)cspline_eq_((double)i);
      temp_lc_point.y = (int)i;

      extra_point.push_back(temp_extra_point);
      extra_point_f.push_back(temp_extra_point);
      center2_point.push_back(temp_center2_point);
      center2_point_f.push_back(temp_center2_point);
      lc_point.push_back(temp_lc_point);
      lc_point_f.push_back(temp_lc_point);
    }
    const Point* extra_points_point_ = (const cv::Point*) Mat(extra_point).data;
    int extra_points_number_ = Mat(extra_point).rows;
    const Point* center2_points_point_ = (const cv::Point*) Mat(center2_point).data;
    int center2_points_number_ = Mat(center2_point).rows;
    const Point* lc_points_point_ = (const cv::Point*) Mat(lc_point).data;
    int lc_points_number_ = Mat(lc_point).rows;

    polylines(_sliding_frame, &extra_points_point_, &extra_points_number_, 1, false, Scalar(255, 100, 100), 10);
    polylines(_sliding_frame, &center2_points_point_, &center2_points_number_, 1, false, Scalar(200, 255, 200), 10);
    polylines(_sliding_frame, &lc_points_point_, &lc_points_number_, 1, false, Scalar(255, 0, 255), 5);
    
    perspectiveTransform(extra_point_f, warped_extra_point, trans);
    perspectiveTransform(center2_point_f, warped_center2_point, trans);
    perspectiveTransform(lc_point_f, warped_lc_point, trans);

    for (int i = 0; i <= height_; i++) {
      Point temp_extra_point;
      Point temp_center2_point;
      Point temp_lc_point;

      temp_extra_point.x = (int)warped_extra_point[i].x;
      temp_extra_point.y = (int)warped_extra_point[i].y;
      temp_center2_point.x = (int)warped_center2_point[i].x;
      temp_center2_point.y = (int)warped_center2_point[i].y;
      temp_lc_point.x = (int)warped_lc_point[i].x;
      temp_lc_point.y = (int)warped_lc_point[i].y;

      extra_points.push_back(temp_extra_point);
      center2_points.push_back(temp_center2_point);
      lc_points.push_back(temp_lc_point);
    }

    const Point* extra_points_point = (const cv::Point*) Mat(extra_points).data;
    int extra_points_number = Mat(extra_points).rows;
    const Point* center2_points_point = (const cv::Point*) Mat(center2_points).data;
    int center2_points_number = Mat(center2_points).rows;
    const Point* lc_points_point = (const cv::Point*) Mat(lc_points).data;
    int lc_points_number = Mat(lc_points).rows;

    Point lane_center2 = *(center2_points_point + center2_points_number - 10);
    Point lane_lc = *(lc_points_point + lc_points_number - 10);

    static Point prev_lane_center2;
    static Point prev_lane_lc;

    gettimeofday(&endTime, NULL);
    if (!flag){
      diffTime = (endTime.tv_sec - start_.tv_sec) + (endTime.tv_usec - start_.tv_usec)/1000000.0;
      flag = true;
    }
    else{
      diffTime = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec)/1000000.0;
      startTime = endTime;
    }
    lane_center2.x = lowPassFilter(diffTime, lane_center2.x, prev_lane_center2.x);
    lane_center2.y = lowPassFilter(diffTime, lane_center2.y, prev_lane_center2.y);
    lane_lc.x = lowPassFilter(diffTime, lane_lc.x, prev_lane_lc.x);
    lane_lc.y = lowPassFilter(diffTime, lane_lc.y, prev_lane_lc.y);

    prev_lane_center2 = lane_center2;
    prev_lane_lc = lane_lc;
    
    //polylines(new_frame, &extra_points_point, &extra_points_number, 1, false, Scalar(0, 255, 255), 5);
    polylines(new_frame, &extra_points_point, &extra_points_number, 1, false, Scalar(255, 100, 100), 10);
    polylines(new_frame, &center2_points_point, &center2_points_number, 1, false, Scalar(100, 255,100), 10);
    polylines(new_frame, &lc_points_point, &lc_points_number, 1, false, Scalar(255, 0, 255), 10);
    
    extra_point.clear();
    center2_point.clear();
    lc_point.clear();
  }  

  if ((!extra2_coef.empty()) && (!left_coef.empty()) && E2_flag == true) {
    mark_ = 2;
    tk::spline cspline_eq_ = cspline(); // s : lane3 coef

    for (int i = 0; i <= height_; i++) {
      Point temp_extra2_point;
      Point temp_center3_point;
      Point temp_lc2_point;

      temp_extra2_point.x = (int)((extra2_coef.at<float>(2, 0) * pow(i, 2)) + (extra2_coef.at<float>(1, 0) * i) + extra2_coef.at<float>(0, 0));
      temp_extra2_point.y = (int)i;
      temp_center3_point.x = (int)((center3_coef.at<float>(2, 0) * pow(i, 2)) + (center3_coef.at<float>(1, 0) * i) + center3_coef.at<float>(0, 0));
      temp_center3_point.y = (int)i;
      temp_lc2_point.x = (int)cspline_eq_((double)i);
      temp_lc2_point.y = (int)i;

      extra2_point.push_back(temp_extra2_point);
      extra2_point_f.push_back(temp_extra2_point);
      center3_point.push_back(temp_center3_point);
      center3_point_f.push_back(temp_center3_point);
      lc2_point.push_back(temp_lc2_point);
      lc2_point_f.push_back(temp_lc2_point);
    }
    const Point* extra2_points_point_ = (const cv::Point*) Mat(extra2_point).data;
    int extra2_points_number_ = Mat(extra2_point).rows;
    const Point* center3_points_point_ = (const cv::Point*) Mat(center3_point).data;
    int center3_points_number_ = Mat(center3_point).rows;
    const Point* lc2_points_point_ = (const cv::Point*) Mat(lc2_point).data;
    int lc2_points_number_ = Mat(lc2_point).rows;

    polylines(_sliding_frame, &extra2_points_point_, &extra2_points_number_, 1, false, Scalar(255, 100, 100), 10);
    polylines(_sliding_frame, &center3_points_point_, &center3_points_number_, 1, false, Scalar(200, 255, 200), 10);
    polylines(_sliding_frame, &lc2_points_point_, &lc2_points_number_, 1, false, Scalar(255, 0, 255), 10);
    
    perspectiveTransform(extra2_point_f, warped_extra2_point, trans);
    perspectiveTransform(center3_point_f, warped_center3_point, trans);
    perspectiveTransform(lc2_point_f, warped_lc2_point, trans);

    for (int i = 0; i <= height_; i++) {
      Point temp_extra2_point;
      Point temp_center3_point;
      Point temp_lc2_point;

      temp_extra2_point.x = (int)warped_extra2_point[i].x;
      temp_extra2_point.y = (int)warped_extra2_point[i].y;
      temp_center3_point.x = (int)warped_center3_point[i].x;
      temp_center3_point.y = (int)warped_center3_point[i].y;
      temp_lc2_point.x = (int)warped_lc2_point[i].x;
      temp_lc2_point.y = (int)warped_lc2_point[i].y;

      extra2_points.push_back(temp_extra2_point);
      center3_points.push_back(temp_center3_point);
      lc2_points.push_back(temp_lc2_point);
    }

    const Point* extra2_points_point = (const cv::Point*) Mat(extra2_points).data;
    int extra2_points_number = Mat(extra2_points).rows;
    const Point* center3_points_point = (const cv::Point*) Mat(center3_points).data;
    int center3_points_number = Mat(center3_points).rows;
    const Point* lc2_points_point = (const cv::Point*) Mat(lc2_points).data;
    int lc2_points_number = Mat(lc2_points).rows;

    Point lane_center3 = *(center3_points_point + center3_points_number - 10);
    Point lane_lc2 = *(lc2_points_point + lc2_points_number - 10);

    static Point prev_lane_center3;
    static Point prev_lane_lc2;

    gettimeofday(&endTime, NULL);
    if (!flag){
      diffTime = (endTime.tv_sec - start_.tv_sec) + (endTime.tv_usec - start_.tv_usec)/1000000.0;
      flag = true;
    }
    else{
      diffTime = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec)/1000000.0;
      startTime = endTime;
    }
    lane_center3.x = lowPassFilter(diffTime, lane_center3.x, prev_lane_center3.x);
    lane_center3.y = lowPassFilter(diffTime, lane_center3.y, prev_lane_center3.y);
    lane_lc2.x = lowPassFilter(diffTime, lane_lc2.x, prev_lane_lc2.x);
    lane_lc2.y = lowPassFilter(diffTime, lane_lc2.y, prev_lane_lc2.y);

    prev_lane_center3 = lane_center3;
    prev_lane_lc2 = lane_lc2;
    
    //polylines(new_frame, &extra2_points_point, &extra2_points_number, 1, false, Scalar(0, 255, 255), 5);
    polylines(new_frame, &extra2_points_point, &extra2_points_number, 1, false, Scalar(255, 100, 100), 10);
    polylines(new_frame, &center3_points_point, &center3_points_number, 1, false, Scalar(100, 255,100), 10);
    polylines(new_frame, &lc2_points_point, &lc2_points_number, 1, false, Scalar(255, 0, 255), 10);
    
    extra2_point.clear();
    center3_point.clear();
    lc2_point.clear();
  }

  if ((left_coef.empty()) && (right_coef.empty())){
    return _frame;
  } 

  return new_frame;
}

void LaneDetector::clear_release() {
  left_lane_inds_.clear();
  right_lane_inds_.clear();
  left_x_.clear();
  left_y_.clear();
  right_x_.clear();
  right_y_.clear();
  extra_x_.clear();
  extra_y_.clear();
  extra2_x_.clear();
  extra2_y_.clear();
  center_x_.clear();
  center_y_.clear();
  center2_x_.clear();
  center2_y_.clear();
  center3_x_.clear();
  center3_y_.clear();
}

void LaneDetector::get_steer_coef(float vel){
  float value;
  if (vel > 1.2f)
    value = 1.2f;
  else
    value = vel;

//  K3_ = (a2_[0] * pow(value, 2)) + (a2_[1] * pow(value, 1)) + a2_[2];
//  K4_ = (b2_[0] * pow(value, 2)) + (b2_[1] * pow(value, 1)) + b2_[2];

  if (value < 0.65f){
    K1_ = K2_ =  K_;
  }
  else{
    K1_ = (a_[0] * pow(value, 4)) + (a_[1] * pow(value, 3)) + (a_[2] * pow(value, 2)) + (a_[3] * value) + a_[4];
    K2_ = (b_[0] * pow(value, 4)) + (b_[1] * pow(value, 3)) + (b_[2] * pow(value, 2)) + (b_[3] * value) + b_[4];
  }
  
}

void LaneDetector::controlSteer() {
  Mat l_fit(left_coef_), r_fit(right_coef_), c_fit(center_coef_), e_fit(extra_coef_), e2_fit(extra2_coef_), c2_fit(center2_coef_), c3_fit(center3_coef_);
  float car_position = width_ / 2;
  float l1 = 0.0f, l2 = 0.0f, l3 = 0.0f;
  float i = ((float)height_) * eL_height_;  
  float j = ((float)height_) * trust_height_;
  float k = ((float)height_) * e1_height_;
  float temp_diff = 10.1f;
  static int wroi_cnt_ = 0;

  lane_coef_.coef.resize(3);
  if (!l_fit.empty() && !r_fit.empty()) {

    lane_coef_.coef[0].a = l_fit.at<float>(2, 0);
    lane_coef_.coef[0].b = l_fit.at<float>(1, 0);
    lane_coef_.coef[0].c = l_fit.at<float>(0, 0);

    lane_coef_.coef[1].a = r_fit.at<float>(2, 0);
    lane_coef_.coef[1].b = r_fit.at<float>(1, 0);
    lane_coef_.coef[1].c = r_fit.at<float>(0, 0);

    lane_coef_.coef[2].a = c_fit.at<float>(2, 0);
    lane_coef_.coef[2].b = c_fit.at<float>(1, 0);
    lane_coef_.coef[2].c = c_fit.at<float>(0, 0);

    l1 =  j - i;
    l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);

    e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //trust_e1
    e_values_[2] = ((lane_coef_.coef[2].a * pow(k, 2)) + (lane_coef_.coef[2].b * k) + lane_coef_.coef[2].c) - car_position;  //e1
    SteerAngle_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
  }

  /*********************/
  /* right lane change */
  /*********************/
  if ((!center2_coef_.empty()) && lc_right_flag_ && wroi_flag_) {
    lane_coef_.coef[0].a = r_fit.at<float>(2, 0);
    lane_coef_.coef[0].b = r_fit.at<float>(1, 0);
    lane_coef_.coef[0].c = r_fit.at<float>(0, 0);

    lane_coef_.coef[1].a = e_fit.at<float>(2, 0);
    lane_coef_.coef[1].b = e_fit.at<float>(1, 0);
    lane_coef_.coef[1].c = e_fit.at<float>(0, 0);

    lane_coef_.coef[2].a = c2_fit.at<float>(2, 0);
    lane_coef_.coef[2].b = c2_fit.at<float>(1, 0);
    lane_coef_.coef[2].c = c2_fit.at<float>(0, 0);

    mark_ = 1;
    tk::spline cspline_eq_ = cspline();

    l3 = (float)cspline_eq_(i) - (float)cspline_eq_(j);
    e_values_[0] = (float)cspline_eq_(i) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l3 / l1));  //trust_e1
    e_values_[2] = (float)cspline_eq_(k)- car_position;  //e1
    SteerAngle2_ = ((-1.0f * K3_) * e_values_[1]) + ((-1.0f * K4_) * e_values_[0]);

    /* cspline follow -> center follow  */
    temp_diff = ((lane_coef_.coef[2].a * pow(height_, 2)) + (lane_coef_.coef[2].b * height_) + lane_coef_.coef[2].c) - (float)cspline_eq_(height_); 
    temp_diff = abs(temp_diff);

    if (center_select_ == 2 && ((0 <= temp_diff) && (temp_diff <= 10))) {
      wroi_cnt_ += 1;
      if (wroi_cnt_ >= 50)  {
        wroi_flag_ = false;
        wroi_cnt_ = 0;
      }
    }
//    if (center_select_ == 2 && ((0 <= temp_diff) && (temp_diff <= 10))) {
//      lc_center_follow_ = true; 
//      l1 =  j - i;
//      l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);
//  
//      e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
//      e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //trust_e1
//      SteerAngle2_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
//    }
  /* cspline follow -> center follow  */
  }
  /********************/
  /* left lane change */
  /********************/
  else if ((!center3_coef_.empty()) && lc_left_flag_ && wroi_flag_) { 
    lane_coef_.coef[0].a = e2_fit.at<float>(2, 0);
    lane_coef_.coef[0].b = e2_fit.at<float>(1, 0);
    lane_coef_.coef[0].c = e2_fit.at<float>(0, 0);

    lane_coef_.coef[1].a = l_fit.at<float>(2, 0);
    lane_coef_.coef[1].b = l_fit.at<float>(1, 0);
    lane_coef_.coef[1].c = l_fit.at<float>(0, 0);

    lane_coef_.coef[2].a = c3_fit.at<float>(2, 0);
    lane_coef_.coef[2].b = c3_fit.at<float>(1, 0);
    lane_coef_.coef[2].c = c3_fit.at<float>(0, 0);

    mark_ = 2;
    tk::spline cspline_eq_ = cspline();

    l3 = (float)cspline_eq_(i) - (float)cspline_eq_(j);
    e_values_[0] = (float)cspline_eq_(i) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l3 / l1));  //trust_e1
    e_values_[2] = (float)cspline_eq_(k)- car_position;  //e1
    SteerAngle2_ = ((-1.0f * K3_) * e_values_[1]) + ((-1.0f * K4_) * e_values_[0]);

    temp_diff = ((lane_coef_.coef[2].a * pow(height_, 2)) + (lane_coef_.coef[2].b * height_) + lane_coef_.coef[2].c) - (float)cspline_eq_(height_); 
    temp_diff = abs(temp_diff);

    if (center_select_ == 3 && ((0 <= temp_diff) && (temp_diff <= 10))) {
      wroi_cnt_ += 1;
      if (wroi_cnt_ >= 50)  {
        wroi_flag_ = false;
        wroi_cnt_ = 0;
      }
    }
    /* cspline follow -> center follow  */
//    if (center_select_ == 3 && ((0 <= temp_diff) && (temp_diff <= 10))) {
//      lc_center_follow_ = true; 
//      l1 =  j - i;
//      l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);
//  
//      e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
//      e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //trust_e1
//      SteerAngle2_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
//    }
    /* cspline follow -> center follow  */
  }
}

tk::spline LaneDetector::cspline() {
  float tY1_ = ((float)height_) * 0;
  float tY2_ = ((float)height_) * 0.1;
  float tY3_ = ((float)height_) * 0;
  float tY4_ = ((float)height_) * 0.1;
  float tY5_ = ((float)height_) * e1_height_;
  
  std::vector<double> X;
  std::vector<double> Y; 
  tk::spline cspline_eq;

  /*****************/
  /* lc_right_coef */
  /*****************/
  if (mark_ == 1 && !right_coef_.empty() && !center2_coef_.empty()) { 
    int tX1_ = (int)((right_coef_.at<float>(2, 0) * pow(tY1_, 2)) + (right_coef_.at<float>(1, 0) * tY1_) + right_coef_.at<float>(0, 0));
    int tX2_ = (int)((right_coef_.at<float>(2, 0) * pow(tY2_, 2)) + (right_coef_.at<float>(1, 0) * tY2_) + right_coef_.at<float>(0, 0));
    int tX3_ = (int)((center2_coef_.at<float>(2, 0) * pow(tY3_, 2)) + (center2_coef_.at<float>(1, 0) * tY3_) + center2_coef_.at<float>(0, 0));
    int tX4_ = (int)((center2_coef_.at<float>(2, 0) * pow(tY4_, 2)) + (center2_coef_.at<float>(1, 0) * tY4_) + center2_coef_.at<float>(0, 0));
  
    if (center_select_ == 1) {
      X = {(double)tX1_, (double)tX2_, (double)(width_/2)};
      //Y = {(double)tY1_, (double)tY2_, (double)tY5_}; //오름차순
      Y = {(double)tY1_, (double)tY2_, (double)height_}; //오름차순
    } 
    else if (center_select_ == 2) {
      X = {(double)tX3_, (double)tX4_, (double)(width_/2)};
      //Y = {(double)tY3_, (double)tY4_, (double)tY5_}; 
      Y = {(double)tY3_, (double)tY4_, (double)height_}; 
    }
    else {
      X = {(double)tX3_, (double)tX4_, (double)(width_/2)};
      //Y = {(double)tY3_, (double)tY4_, (double)tY5_}; 
      Y = {(double)tY3_, (double)tY4_, (double)height_}; 
    }
  
    tk::spline s(Y, X, tk::spline::cspline); 
    cspline_eq = s;
  }
  /****************/
  /* lc_left_coef */
  /****************/
  else if (mark_ == 2 && !left_coef_.empty() && !center3_coef_.empty()) { 
    int tX1_ = (int)((left_coef_.at<float>(2, 0) * pow(tY1_, 2)) + (left_coef_.at<float>(1, 0) * tY1_) + left_coef_.at<float>(0, 0));
    int tX2_ = (int)((left_coef_.at<float>(2, 0) * pow(tY2_, 2)) + (left_coef_.at<float>(1, 0) * tY2_) + left_coef_.at<float>(0, 0));
    int tX3_ = (int)((center3_coef_.at<float>(2, 0) * pow(tY3_, 2)) + (center3_coef_.at<float>(1, 0) * tY3_) + center3_coef_.at<float>(0, 0));
    int tX4_ = (int)((center3_coef_.at<float>(2, 0) * pow(tY4_, 2)) + (center3_coef_.at<float>(1, 0) * tY4_) + center3_coef_.at<float>(0, 0));
  
    if (center_select_ == 1) {
      X = {(double)tX1_, (double)tX2_, (double)(width_/2)};
      //Y = {(double)tY1_, (double)tY2_, (double)tY5_}; //오름차순
      Y = {(double)tY1_, (double)tY2_, (double)height_}; //오름차순
    } 
    else if (center_select_ == 2) {
      X = {(double)tX3_, (double)tX4_, (double)(width_/2)};
      //Y = {(double)tY3_, (double)tY4_, (double)tY5_}; 
      Y = {(double)tY3_, (double)tY4_, (double)height_}; 
    }
    else {
      X = {(double)tX3_, (double)tX4_, (double)(width_/2)};
      //Y = {(double)tY3_, (double)tY4_, (double)tY5_}; 
      Y = {(double)tY3_, (double)tY4_, (double)height_}; 
    }
  
    tk::spline s(Y, X, tk::spline::cspline); 
    cspline_eq = s;
  }

  return cspline_eq;
}

Mat LaneDetector::drawBox(Mat frame)
{
  std::string name;
  Scalar color;
  int x, y, w, h;

  if(imageStatus_) {
    name = name_;
    x = x_;
    y = y_;
    w = w_;
    h = h_;
  }
  else if(rearImageStatus_) {
    name = r_name_;
    x = rx_;
    y = ry_;
    w = rw_;
    h = rh_;
  }

  if (name == "SV1") color = cv::Scalar(255, 0, 255);
  else if (name == "SV2") color = cv::Scalar(55, 200, 55);

  Size text_size = getTextSize(name, FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
  int max_width = (text_size.width > w + 2) ? text_size.width : (w + 2);
  max_width = max(max_width, (int)w + 2);
  rectangle(frame, Rect(x, y, w, h), color, 2);
  rectangle(frame, Point2f(max((int)x - 1, 0), max((int)y - 35, 0)), Point2f(min((int)x + max_width, frame.cols - 1), min((int)y, frame.rows - 1)), color, -1, 8, 0);
  putText(frame, name, Point2f(x, y-16), FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(0,0,0), 2);
  return frame;
}

Mat LaneDetector::estimateDistance(Mat frame, Mat trans, double cycle_time, bool _view){
  Mat res_frame;
  Point warp_center;
  static Point prev_warp_center;
  int dist_pixel = 0;
  float est_dist = 0.f, original_est_vel = 0.f;
  static float prev_dist = 0.f, prev_est_vel = 0.f;

  frame.copyTo(res_frame);
  if(imageStatus_) center_ = Point((x_ + w_ / 2), (y_ + h_)); // front-cam yolo
  else center_ = Point(rx_ + rw_ / 2, ry_ + rh_); // rear-cam yolo 
  warp_center = warpPoint(center_, trans);
  warp_center.x = lowPassFilter(cycle_time, warp_center.x, prev_warp_center.x);
  warp_center.y = lowPassFilter(cycle_time, warp_center.y, prev_warp_center.y);
  prev_warp_center = warp_center;
  warp_center_ = warp_center;
  dist_pixel = warp_center.y;

  if (imageStatus_){
    est_dist = 2.50f - (dist_pixel/frontRoi_ratio); //front camera
    if (est_dist > 0.30f && est_dist < 2.50f) {
      est_dist_ = est_dist;
    }
    else 
      est_dist_ = 0.0f;
  }
  else{
    est_dist = 2.50f - (dist_pixel/rearRoi_ratio); //rear camera
    if (est_dist > 0.30f && est_dist < 2.50f) {
      est_dist_ = est_dist;
    }
    else 
      est_dist_ = 0.0f;
  }

  if (prev_dist == 0){
    prev_dist = est_dist_;
  } 

  if (est_dist_ != 0) {
    est_dist_ = lowPassFilter(cycle_time, est_dist_, prev_dist);
    original_est_vel = ((prev_dist - est_dist_) / cycle_time) + cur_vel_;
    est_vel_ = lowPassFilter2(cycle_time, original_est_vel, prev_est_vel);

    prev_est_vel = est_vel_;
    prev_dist = est_dist_; 
  }

  return res_frame;
}

float LaneDetector::display_img(Mat _frame, int _delay, bool _view) {   
  Mat test_trans, new_frame, gray_frame, binary_frame, overlap_frame, sliding_frame, resized_frame, warped_frame;
  static struct timeval startTime, endTime;
  static bool flag = false;
  static bool lc_flag = false;
  double diffTime = 0.0;
  
  /* apply ROI setting */
  if(imageStatus_) {
    if((lc_left_flag || lc_right_flag) && lc_flag == false) {
      lc_flag = true;
      wroi_flag_ = true;
    }
    else if (lc_left_flag == false && lc_right_flag == false) {
      lc_flag = false;
      wroi_flag_ = false;
    }

    if(lc_flag && wroi_flag_) {
      if(lc_right_flag) { // right lane change mode
        std::copy(rROIcorners_.begin(), rROIcorners_.end(), corners_.begin());
        std::copy(rROIwarpCorners_.begin(), rROIwarpCorners_.end(), warpCorners_.begin());
        lc_right_flag_ = true;
        lc_center_follow_ = false;
      }
      else if(lc_left_flag) { // left lane change mode
        std::copy(lROIcorners_.begin(), lROIcorners_.end(), corners_.begin());
        std::copy(lROIwarpCorners_.begin(), lROIwarpCorners_.end(), warpCorners_.begin());
        lc_left_flag_ = true;
        lc_center_follow_ = false;
      }
    }
    else { // normal mode
      std::copy(fROIcorners_.begin(), fROIcorners_.end(), corners_.begin());
      std::copy(fROIwarpCorners_.begin(), fROIwarpCorners_.end(), warpCorners_.begin());
 //     std::copy(lROIcorners_.begin(), lROIcorners_.end(), corners_.begin()); // FOR ICRA
   //   std::copy(lROIwarpCorners_.begin(), lROIwarpCorners_.end(), warpCorners_.begin());
      lc_right_flag_ = false; 
      lc_left_flag_ = false; 
      lc_center_follow_ = true;
    }
  }
  else if(rearImageStatus_ == true) {
    map1_ = r_map1_.clone();
    map2_ = r_map2_.clone();
    std::copy(rearROIcorners_.begin(), rearROIcorners_.end(), corners_.begin());
    std::copy(rearROIwarpCorners_.begin(), rearROIwarpCorners_.end(), warpCorners_.begin());
    lc_right_flag_ = false; 
    lc_left_flag_ = false;  
    lc_center_follow_ = true;
  }

  Mat trans = getPerspectiveTransform(corners_, warpCorners_);

  //For Estimation ROI
  std::copy(testROIcorners_.begin(), testROIcorners_.end(), test_corners_.begin());
  std::copy(testROIwarpCorners_.begin(), testROIwarpCorners_.end(), test_warpCorners_.begin());
  test_trans = getPerspectiveTransform(test_corners_, test_warpCorners_);

  /* End apply ROI setting */

//  if(!_frame.empty()) resize(_frame, new_frame, Size(width_, height_), 0, 0, cv::INTER_LINEAR);
//
//  cuda::GpuMat gpu_map1, gpu_map2;
//  gpu_map1.upload(map1_);
//  gpu_map2.upload(map2_);
//  
//  cuda::GpuMat gpu_frame, gpu_remap_frame, gpu_warped_frame, gpu_blur_frame, gpu_gray_frame, gpu_binary_frame;
//  
//  gpu_frame.upload(new_frame);
//  cuda::remap(gpu_frame, gpu_remap_frame, gpu_map1, gpu_map2, INTER_LINEAR);
//  gpu_remap_frame.download(new_frame);  /* apply camera matrix to new_frame */
//
//  /* ROI apply frame */
//  if (imageStatus_ && TEST) { // For view TEST ROI
//    cuda::warpPerspective(gpu_remap_frame, gpu_warped_frame, test_trans, Size(width_, height_));
//  }
//  else { 
//    cuda::warpPerspective(gpu_remap_frame, gpu_warped_frame, trans, Size(width_, height_)); 
//  }
//  gpu_warped_frame.download(warped_frame);
//
//  static cv::Ptr< cv::cuda::Filter > filters;
//  filters = cv::cuda::createGaussianFilter(gpu_warped_frame.type(), gpu_blur_frame.type(), cv::Size(5,5), 0, 0, cv::BORDER_DEFAULT);
//  filters->apply(gpu_warped_frame, gpu_blur_frame);
//  cuda::cvtColor(gpu_blur_frame, gpu_gray_frame, COLOR_BGR2GRAY);
//  gpu_gray_frame.download(gray_frame);
//  
//  for(int y = height_/2; y < gray_frame.rows; y++) {
//      for(int x = 0; x < gray_frame.cols; x++) {
//          if(gray_frame.at<uchar>(y, x) == 0) {
//              gray_frame.at<uchar>(y, x) = 0;
//          }
//      }
//  }
  if (!_frame.empty())
      resize(_frame, new_frame, Size(width_, height_), 0, 0, cv::INTER_LINEAR);
  
  cv::Mat map1, map2;
  map1 = Mat(map1_);
  map2 = Mat(map2_);
  
  cv::Mat remap_frame;
  remap(new_frame, remap_frame, map1, map2, INTER_LINEAR);
  
//  cv::Mat warped_frame;
  if (imageStatus_ && TEST) { // For view TEST ROI
      warpPerspective(remap_frame, warped_frame, test_trans, Size(width_, height_));
  } else {
      warpPerspective(remap_frame, warped_frame, trans, Size(width_, height_));
  }
  
  cv::Mat blur_frame; 
  GaussianBlur(warped_frame, blur_frame, Size(5, 5), 0, 0, BORDER_DEFAULT);
//  cv::Mat gray_frame;
  cvtColor(blur_frame, gray_frame, COLOR_BGR2GRAY);
  
  for (int y = height_ / 2; y < gray_frame.rows; y++) {
      for (int x = 0; x < gray_frame.cols; x++) {
          if (gray_frame.at<uchar>(y, x) == 0) {
              gray_frame.at<uchar>(y, x) = 0;
          }
      }
  }
//  cv::Mat binary_frame;
  /* adaptive Threshold */
  adaptiveThreshold(gray_frame, binary_frame, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, Threshold_box_size_, -(Threshold_box_offset_));


//  if(prev_frame.empty()) {
//    prev_frame = binary_frame;
//  }
//  cv::addWeighted(binary_frame, 1, prev_frame, 1, 0.0, overlap_frame);
//  prev_frame = binary_frame;
//
//  sliding_frame = detect_lines_sliding_window(overlap_frame, _view);

  /* estimate Distance */
  if ((((x_ > 0 && x_ < 640) && \
          (y_ > 0 && y_ < 480) && \
          (w_ > 0 && w_ < 640) && \
          (h_ > 0 && h_ < 480))) || \
      (((rx_ > 0 && rx_ < 640) && \
        (ry_ > 0 && ry_ < 480) && \
        (rw_ > 0 && rw_ < 640) && \
        (rh_ > 0 && rh_ < 480)))) 
  {
    gettimeofday(&endTime, NULL);
    if (!flag){
      diffTime = (endTime.tv_sec - start_.tv_sec) + (endTime.tv_usec - start_.tv_usec)/1000000.0;
      flag = true;
    }
    else{
      diffTime = (endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec)/1000000.0;
      startTime = endTime;
    }
    if (imageStatus_) sliding_frame = estimateDistance(sliding_frame, test_trans, diffTime, _view);
    else if (rearImageStatus_) sliding_frame = estimateDistance(sliding_frame, trans, diffTime, _view);

    if(imageStatus_ && est_dist_ != 0) {
      //distance_ = (int)((2.50f - est_dist_)*frontRoi_ratio); // FOR ICRA
    }
    else if(rearImageStatus_ && est_dist_ != 0) {
      distance_ = (int)((2.50f - est_dist_)*rearRoi_ratio); // FOR ICRA
    }
  }
  else {
//    est_dist_ = 0.0f; // box튀면 값이 0됨 (LPF로 처리도 안됨) 
//    est_vel_ = 0.0f;

    if(rearImageStatus_) {
      distance_ = 10.0f; 
    }
  }

  sliding_frame = detect_lines_sliding_window(binary_frame, _view);
  controlSteer();

  if (_view) {
    resized_frame = draw_lane(sliding_frame, new_frame);
    namedWindow("Window1");
    moveWindow("Window1", 0, 0);
    namedWindow("Window2");
    moveWindow("Window2", 710, 0);
    namedWindow("Window3");
    moveWindow("Window3", 1340, 0);
    namedWindow("Histogram Clusters");
    moveWindow("Histogram Clusters", 710, 700);

    if(!new_frame.empty()) {
      if(TEST) cv::circle(new_frame, center_, 10, (0,0,255), -1);
      new_frame = drawBox(new_frame);
      imshow("Window1", new_frame);
    }
    if(!sliding_frame.empty()) {
      if(TEST) cv::circle(sliding_frame, warp_center_, 10, (0,0,255), -1);
      imshow("Window2", sliding_frame);
     // imshow("Window2", warped_frame);
    }
    if(!resized_frame.empty()){
      imshow("Window3", resized_frame);
    }
//    if(!cluster_frame.empty()){
 //     resize(cluster_frame, cluster_frame, Size(640, 480));
  //    imshow("Histogram Clusters", cluster_frame);
  //  }

    waitKey(_delay);
  }
  clear_release();


  

  return SteerAngle_;
}

} /* namespace lane_detect */
