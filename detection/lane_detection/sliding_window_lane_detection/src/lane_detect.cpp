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
  std::string ImageSubTopicName;
  int ImageSubQueueSize;

  std::string XavPubTopicName;
  int XavPubQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("subscribers/image_to_lane/topic", ImageSubTopicName, std::string("usb_cam/image_raw"));
  this->get_parameter_or("subscribers/image_to_lane/queue_size", ImageSubQueueSize, 1);

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

  ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(ImageSubTopicName, qos, std::bind(&LaneDetector::ImageSubCallback, this, std::placeholders::_1));
  DistanceSubscriber_ = this->create_subscription<ros2_msg::msg::Obj2xav>("min_distance", 10, std::bind(&LaneDetector::DistanceSubCallback, this, std::placeholders::_1));

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  XavPublisher_ = this->create_publisher<ros2_msg::msg::Lane2xav>(XavPubTopicName, XavPubQueueSize);
  /***************/
  /* View Option */
  /***************/
  this->get_parameter_or("image_view/enable_opencv", viewImage_, true);
  this->get_parameter_or("image_view/wait_key_delay", waitKeyDelay_, 3);

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


  /*** front cam calibration  ***/
  f_camera_matrix = Mat::eye(3, 3, CV_64FC1);
  f_dist_coeffs = Mat::zeros(1, 5, CV_64FC1);
  f_camera_matrix = (Mat1d(3, 3) << f_matrix[0], f_matrix[1], f_matrix[2], f_matrix[3], f_matrix[4], f_matrix[5], f_matrix[6], f_matrix[7], f_matrix[8]);
  f_dist_coeffs = (Mat1d(1, 5) << f_dist_coef[0], f_dist_coef[1], f_dist_coef[2], f_dist_coef[3], f_dist_coef[4]);
  initUndistortRectifyMap(f_camera_matrix, f_dist_coeffs, Mat(), f_camera_matrix, Size(640, 480), CV_32FC1, f_map1_, f_map2_);


  map1_ = f_map1_.clone();
  map2_ = f_map2_.clone();
  int width = 640;   // image width 
  int height = 480;  // image height

  this->get_parameter_or("ROI/width", width_, 640);
  this->get_parameter_or("ROI/height", height_, 480);

  float t_gap[5], b_gap[5], t_height[5], b_height[5], f_extra[5], b_extra[5];
  int top_gap[5], bot_gap[5], top_height[5], bot_height[5], extra_up[5], extra_down[5];
  float wide_extra_upside_[5], wide_extra_downside_[5];

  this->get_parameter_or("ROI/dynamic_roi",option_, true);
  this->get_parameter_or("ROI/threshold",threshold_, 128);

  this->get_parameter_or("ROI/front_cam/top_gap",t_gap[0], 0.336f);
  this->get_parameter_or("ROI/front_cam/bot_gap",b_gap[0], 0.078f);
  this->get_parameter_or("ROI/front_cam/top_height",t_height[0], 0.903f);
  this->get_parameter_or("ROI/front_cam/bot_height",b_height[0], 0.528f);
  this->get_parameter_or("ROI/front_cam/extra_f",f_extra[0], 0.0f);
  this->get_parameter_or("ROI/front_cam/extra_b",b_extra[0], 0.0f);
  this->get_parameter_or("ROI/front_cam/extra_up",extra_up[0], 0);
  this->get_parameter_or("ROI/front_cam/extra_down",extra_down[0], 0);

  this->get_parameter_or("threshold/box_size", Threshold_box_size_, 51);
  this->get_parameter_or("threshold/box_offset", Threshold_box_offset_, 50);
  distance_ = 0;

  corners_.resize(4);
  warpCorners_.resize(4);

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

  /*  Synchronisation         */
  cam_new_frame_arrived = false;

  isNodeRunning_ = true;
  lanedetect_Thread = std::thread(&LaneDetector::lanedetectInThread, this);

}

LaneDetector::~LaneDetector(void) 
{
  isNodeRunning_ = false;

  /*  Unblock the other thread to shutdown the programm smoothly  */
  cam_new_frame_arrived = true;
  cam_condition_variable.notify_one();

  ros2_msg::msg::Lane2xav xav;
  xav.coef = lane_coef_.coef;

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

  while(!imageStatus_) {
    /*  Synchronize at startup  */
      unique_lock<mutex> lock(cam_mutex);
      if(cam_condition_variable.wait_for(lock, wait_duration, [this] { return (imageStatus_); } )) {
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
  while(!controlDone_ && rclcpp::ok()) 
  {
    struct timeval start_time, end_time, cur_time;
    gettimeofday(&start_time, NULL);

    if(imageStatus_) { /* use front_cam  */

      /*  Synchronize ImageSubCallback and this thread  */
      {
        unique_lock<mutex> lock(cam_mutex);
        cam_condition_variable.wait(lock, [this] { return cam_new_frame_arrived; } );
        cam_new_frame_arrived = false;
      }
      display_img(camImageCopy_, waitKeyDelay_, viewImage_);
    
      xav.coef = lane_coef_.coef; 
      XavPublisher_->publish(xav);
    }

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
  }
}

void LaneDetector::ImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  static cv::Mat prev_img;

  cv_bridge::CvImagePtr cam_image;
  try{
    cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception : %s", e.what());
  }

  if(!cam_image->image.empty()) {

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
void LaneDetector::DistanceSubCallback(const ros2_msg::msg::Obj2xav::SharedPtr msg) {
  int droi_distance = (int)((10.0f - msg->min_dist)*45.33f);
  dist_mutex_.lock();
  distance_ = droi_distance;
  dist_mutex_.unlock();
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

  int window_width = margin * 2;  // 120
  int window_height;
  dist_mutex_.lock();
  int distance = distance_;
  dist_mutex_.unlock();
  L_flag = true;
  R_flag = true;
  E_flag = true;
  E2_flag = true;
  if(!lc_right_flag_) E_flag = false; // extra right lane is only need for right lc flag 
  if(!lc_left_flag_) E2_flag = false; // extra left lane is only need for left lc flag 
  
  if (option_) {
    window_height = (height >= distance) ? ((height-distance) / n_windows) : (height / n_windows);  // defalut = 53
  } else {
    distance = 0;
    window_height = height / n_windows;
  }
  
  int Llane_base = arrMaxIdx(hist, 100, mid_point, width); // 140 ~ mid
  int Rlane_base = arrMaxIdx(hist, mid_point, width-100, width); // mid ~ w-140
  int cluster_num = 2;
  std::vector<int> maxIndices;

  int Llane_current = Llane_base;
  int Rlane_current = Rlane_base;
  int L_prev =  Llane_current;
  int R_prev =  Rlane_current;
  int L_gap = 0;
  int R_gap = 0;
  static int prev_L_gap = 0;
  static int prev_R_gap = 0;
  static int prev_Llane_current = 0;
  static int prev_Rlane_current = 0;

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
//  RCLCPP_INFO(this->get_logger(), "E2lane | Llane | Rlane | Elane | E2_flag | E_flag : %d | %d | %d | %d | %d | %d \n", E2lane_base, Llane_base, Rlane_base, Elane_base, E2_flag, E_flag);

  for (int window = 0; window < n_windows; window++) {
    int  Ly_pos = height - (window + 1) * window_height - 1; // win_y_low , win_y_high = win_y_low - window_height
    int  Ry_pos = height - (window + 1) * window_height - 1;
    int  Ly_top = height - window * window_height;
    int  Ry_top = height - window * window_height;

    int  Lx_pos = Llane_current - margin; // win_xleft_low, win_xleft_high = win_xleft_low + margin*2
    int  Rx_pos = Rlane_current - margin; // win_xrignt_low, win_xright_high = win_xright_low + margin*2
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
    }
    int nZ_y, nZ_x;
    good_left_inds.clear();
    good_right_inds.clear();

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
          }
          good_right_inds.push_back(index);
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
        Rlane_current += (R_gap);
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
    }
    /* center1  */
    if ((Lsum != 0) && (Rsum != 0)) {
	    int c_cnt = 0;
	    int l_cnt = 0;
	    int r_cnt = 0;
      for (int i = 0; i < Llane_x.size() ; i++) {
          if((Llane_x.at(i) != -1) && (Rlane_x.at(i) != -1)) {
              center_x_.insert(center_x_.end(), (Llane_x.at(i)+Rlane_x.at(i)) / 2 );
              center_y_.insert(center_y_.end(), Llane_y.at(i));
          } 
      }
    }
    L_prev = Llane_current;
    R_prev = Rlane_current;
  }
  
  if (left_x_.size() != 0) {
    left_coef_ = polyfit(left_y_, left_x_);
  }
  if (right_x_.size() != 0) {
    right_coef_ = polyfit(right_y_, right_x_);
  }
  
  int center_diff_ = 999;
  if (left_x_.size() != 0 && right_x_.size() != 0) { // && !center_x_.empty() remove
      center_coef_ = (left_coef_ + right_coef_)/2;
      center_diff_ = abs(mid_point ); // - center_x_.front() remove
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

Point LaneDetector::warpPoint(Point center, Mat trans){
  Point warp_center, avg_center;

  warp_center.x = (trans.at<double>(0,0)*center.x + trans.at<double>(0,1)*center.y + trans.at<double>(0,2)) / (trans.at<double>(2,0)*center.x + trans.at<double>(2,1)*center.y + trans.at<double>(2,2));
  warp_center.y = (trans.at<double>(1,0)*center.x + trans.at<double>(1,1)*center.y + trans.at<double>(1,2)) / (trans.at<double>(2,0)*center.x + trans.at<double>(2,1)*center.y + trans.at<double>(2,2));

  return warp_center;
}

Mat LaneDetector::draw_lane(Mat _sliding_frame, Mat _frame) {
  Mat new_frame, left_coef(left_coef_), right_coef(right_coef_), center_coef(center_coef_), trans;

  static struct timeval endTime, startTime;
  static bool flag;
  double diffTime;

  trans = getPerspectiveTransform(fROIwarpCorners_, fROIcorners_);
  _frame.copyTo(new_frame);

  vector<Point> left_point;
  vector<Point> right_point;
  vector<Point> center_point;

  vector<Point2f> left_point_f;
  vector<Point2f> right_point_f;
  vector<Point2f> center_point_f;

  vector<Point2f> warped_left_point;
  vector<Point2f> warped_right_point;
  vector<Point2f> warped_center_point;

  vector<Point> left_points;
  vector<Point> right_points;
  vector<Point> center_points;

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
    if(R_flag == true)
      //polylines(new_frame, &right_points_point, &right_points_number, 1, false, Scalar(100, 100, 255), 5);
      polylines(new_frame, &right_points_point, &right_points_number, 1, false, Scalar(255, 100, 100), 10);
    if(L_flag == true && R_flag == true)
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
    dist_mutex_.lock();
    temp_droi_point.y = distance_;
    dist_mutex_.unlock();
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
      
      if(imageStatus_) {
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
  center_x_.clear();
  center_y_.clear();
}

void LaneDetector::get_lane_coef() {
  Mat l_fit(left_coef_), r_fit(right_coef_), c_fit(center_coef_);

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
  }
}

float LaneDetector::display_img(Mat _frame, int _delay, bool _view) {   
  Mat new_frame, gray_frame, binary_frame, overlap_frame, sliding_frame, resized_frame, warped_frame;
  static struct timeval startTime, endTime;
  static bool flag = false;
  double diffTime = 0.0;
  
  /* apply ROI setting */
  if(imageStatus_) {
    std::copy(fROIcorners_.begin(), fROIcorners_.end(), corners_.begin());
    std::copy(fROIwarpCorners_.begin(), fROIwarpCorners_.end(), warpCorners_.begin());
    lc_right_flag_ = false; 
    lc_left_flag_ = false; 
  }

  Mat trans = getPerspectiveTransform(corners_, warpCorners_);

  /* End apply ROI setting */
  if (!_frame.empty())
      resize(_frame, new_frame, Size(width_, height_), 0, 0, cv::INTER_LINEAR);
  
  cv::Mat map1, map2;
  map1 = Mat(map1_);
  map2 = Mat(map2_);
  
  cv::Mat remap_frame;
  remap(new_frame, remap_frame, map1, map2, INTER_LINEAR);
  
//  cv::Mat warped_frame;
  if (imageStatus_) { // For view TEST ROI
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

  sliding_frame = detect_lines_sliding_window(binary_frame, _view);
  get_lane_coef();
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
      imshow("Window1", new_frame);
    }
    if(!sliding_frame.empty()) {
      imshow("Window2", sliding_frame);
    }
    if(!resized_frame.empty()){
      imshow("Window3", resized_frame);
    }

    waitKey(_delay);
  }
  clear_release();

}

} /* namespace lane_detect */
