#include "lane_detect.hpp"
#define BATCH_SIZE 1
static const int INPUT_C = 3;
static const int INPUT_H = 288;
static const int INPUT_W = 800;
static const int OUTPUT_C = 101;
static const int OUTPUT_H = 27;
static const int OUTPUT_W = 4;
static const int OUTPUT_SIZE = OUTPUT_C * OUTPUT_H * OUTPUT_W;
const char* INPUT_BLOB_NAME = "data";
const char* OUTPUT_BLOB_NAME = "prob";
static Logger gLogger;


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
  DistanceSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("min_distance", 10, std::bind(&LaneDetector::DistanceSubCallback, this, std::placeholders::_1));

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
  std::ifstream file("lane_det.engine", std::ios::binary);
  if (file.good()) {
      file.seekg(0, file.end);
            size = file.tellg();
            file.seekg(0, file.beg);
            trtModelStream = new char[size];
            assert(trtModelStream);
            file.read(trtModelStream, size);
            file.close();
        }
        else {
            RCLCPP_INFO(this->get_logger(), "NONO");
        }
            runtime = createInferRuntime(gLogger);
            assert(runtime != nullptr);
            engine = runtime->deserializeCudaEngine(trtModelStream, size);
            assert(engine != nullptr);
            context = engine->createExecutionContext();
            assert(context != nullptr);
            delete[] trtModelStream;
        // Initialize subscriber to the image topic

        RCLCPP_INFO(this->get_logger(), "Initialize Finish.");
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

void LaneDetector::doInference(IExecutionContext& context, float* input, float* output, int batchSize) {
    const ICudaEngine& engine = context.getEngine();

    // Pointers to input and output device buffers to pass to engine.
    // Engine requires exactly IEngine::getNbBindings() number of buffers.
    assert(engine.getNbBindings() == 2);
    void* buffers[2];

    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine.getBindingIndex(INPUT_BLOB_NAME);
    const int outputIndex = engine.getBindingIndex(OUTPUT_BLOB_NAME);

    // Create GPU buffers on device
    CHECK(cudaMalloc(&buffers[inputIndex], batchSize * INPUT_C * INPUT_H * INPUT_W * sizeof(float)));
    CHECK(cudaMalloc(&buffers[outputIndex], batchSize * OUTPUT_SIZE * sizeof(float)));

    // Create stream
    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));

    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CHECK(cudaMemcpyAsync(buffers[inputIndex], input, batchSize * INPUT_C * INPUT_H * INPUT_W * sizeof(float),
          cudaMemcpyHostToDevice, stream));
    context.enqueue(batchSize, buffers, stream, nullptr);
    CHECK(cudaMemcpyAsync(output, buffers[outputIndex], batchSize * OUTPUT_SIZE * sizeof(float),
          cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CHECK(cudaFree(buffers[inputIndex]));
    CHECK(cudaFree(buffers[outputIndex]));
}

void LaneDetector::softmax_mul(float* x, float* y, int rows, int cols, int chan)
{
    for(int i = 0, wh = rows * cols; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            float sum = 0.0;
            float sum_tmp = 0.0;
            float thresh_hold = 0.9;
            float expect = 0.0;
            for(int k = 0; k < chan - 1; k++)
            {
                
                x[k * wh + i * cols + j] = exp(x[k * wh + i * cols + j]);
                sum += x[k * wh + i * cols + j];
            }
           
            for(int k = 0; k < chan - 1; k++)
            {
                x[k * wh + i * cols + j] /= sum;
                if(x[k * wh + i * cols + j] > 0.1 && !option_) sum_tmp += x[k * wh + i * cols + j];
            }
            
            for(int k = 0; k < chan - 1; k++)
            {
                x[k * wh + i * cols + j] = x[k * wh + i * cols + j] * (k + 1);
                expect += x[k * wh + i * cols + j];
            }

            if(sum_tmp < thresh_hold && !option_ )  y[i * cols + j] = 1;
            else y[i * cols + j] = expect;
        }
    }
}
void LaneDetector::argmax(float* x, float* y, int rows, int cols, int chan)
{
    for(int i = 0,wh = rows * cols; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            int max = -10000000;
            int max_ind = -1;
            for(int k = 0; k < chan; k++)
            {
                if(x[k * wh + i * cols + j] > max)
                {
                    max = x[k * wh + i * cols + j];
                    max_ind = k;
                }
            }
            y[i * cols + j] = max_ind;
        }
    }
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
void LaneDetector::DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
 // int droi_distance = (int)((10.0f - msg->data)*45.33f);
 // dist_mutex_.lock();
 // distance_ = droi_distance;
 // dist_mutex_.unlock();
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
  Mat l_fit(left_coef_), r_fit(right_coef_), c_fit(center_coef_), er_fit(extra_right_coef_), ec_fit(extra_center_coef_);

  lane_coef_.coef.resize(8);
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


    lane_coef_.coef[3].a = er_fit.at<float>(2, 0);
    lane_coef_.coef[3].b = er_fit.at<float>(1, 0);
    lane_coef_.coef[3].c = er_fit.at<float>(0, 0);

    lane_coef_.coef[4].a = ec_fit.at<float>(2, 0);
    lane_coef_.coef[4].b = ec_fit.at<float>(1, 0);
    lane_coef_.coef[4].c = ec_fit.at<float>(0, 0);


    //original_coef
    lane_coef_.coef[5].a = orig_left_coef_.at<float>(2, 0);
    lane_coef_.coef[5].b = orig_left_coef_.at<float>(1, 0);
    lane_coef_.coef[5].c = orig_left_coef_.at<float>(0, 0);

    lane_coef_.coef[6].a = orig_right_coef_.at<float>(2, 0);
    lane_coef_.coef[6].b = orig_right_coef_.at<float>(1, 0);
    lane_coef_.coef[6].c = orig_right_coef_.at<float>(0, 0);

    lane_coef_.coef[7].a = orig_extra_right_coef_.at<float>(2, 0);
    lane_coef_.coef[7].b = orig_extra_right_coef_.at<float>(1, 0);
    lane_coef_.coef[7].c = orig_extra_right_coef_.at<float>(0, 0);
  }
}

float LaneDetector::display_img(Mat _frame, int _delay, bool _view) {   
  Mat new_frame, gray_frame, binary_frame, overlap_frame, sliding_frame, resized_frame, warped_frame,blended;
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
  Mat reverse_trans = getPerspectiveTransform(warpCorners_ , corners_); //BEV to Original Image
  /* End apply ROI setting */
  if (!_frame.empty())
      resize(_frame, new_frame, Size(width_, height_), 0, 0, cv::INTER_LINEAR);
  
//  cv::Mat warped_frame;
  if (imageStatus_) { // For view TEST ROI
      warpPerspective(_frame, warped_frame, trans, Size(width_, height_),INTER_LINEAR);
  }
  
  
    /* Ultra Fast Lane detection */
      //preprocessing
    static float data[BATCH_SIZE * INPUT_C * INPUT_H * INPUT_W];
    static float prob[BATCH_SIZE * OUTPUT_SIZE];


    std::vector<float> result(INPUT_C * INPUT_W * INPUT_H);
    
    cv::Mat resized;
    cv::resize(_frame, resized, cv::Size(INPUT_W, INPUT_H));
    Mat orig_img = _frame.clone();
    cv::Mat img_float;

    resized.convertTo(img_float, CV_32FC3, 1. / 255.);

    // HWC TO CHW
    std::vector<cv::Mat> input_channels(INPUT_C);
    cv::split(img_float, input_channels);

    // normalize
  
    auto data_ = result.data();
    int channelLength = INPUT_H * INPUT_W;
    static float mean[]= {0.485, 0.456, 0.406};
    static float std[] = {0.229, 0.224, 0.225};
    for (int i = 0; i < INPUT_C; ++i) {
        cv::Mat normed_channel = (input_channels[i] - mean[i]) / std[i];
        memcpy(data_, normed_channel.data, channelLength * sizeof(float));
        data_ += channelLength;
    }
    memcpy(data, &result[0], INPUT_C * INPUT_W * INPUT_H * sizeof(float));
    //inference
    doInference(*context, data, prob, BATCH_SIZE); //prob: size (101, 56, 4)
    //post processing
    float max_ind[BATCH_SIZE * OUTPUT_H * OUTPUT_W];
    float prob_reverse[BATCH_SIZE * OUTPUT_SIZE];
    /* do out_j = out_j[:, ::-1, :] in python list*/
    float expect[BATCH_SIZE * OUTPUT_H * OUTPUT_W];
    for (int k = 0, wh = OUTPUT_W * OUTPUT_H; k < OUTPUT_C; k++) {
        for(int j = 0; j < OUTPUT_H; j ++) {
            for(int l = 0; l < OUTPUT_W; l++) {
            prob_reverse[k * wh + (OUTPUT_H - 1 - j) * OUTPUT_W + l] =
            prob[k * wh + j * OUTPUT_W + l];
            }
        }
    }

    argmax(prob_reverse, max_ind, OUTPUT_H, OUTPUT_W, OUTPUT_C);
    /* calculate softmax and Expect */
    softmax_mul(prob_reverse, expect, OUTPUT_H, OUTPUT_W, OUTPUT_C);
    for(int k = 0; k < OUTPUT_H; k++) {
        for(int j = 0; j < OUTPUT_W; j++) {
            max_ind[k * OUTPUT_W + j] == 100 ? expect[k * OUTPUT_W + j] = 0 :
            expect[k * OUTPUT_W + j] = expect[k * OUTPUT_W + j];
        }
    }
    std::vector<int> i_ind;
    for(int k = 0; k < OUTPUT_W; k++) {
        int ii = 0;
        for(int g = 0; g < OUTPUT_H; g++) {
            if(expect[g * OUTPUT_W + k] != 0)
                ii++;
            }
            if(ii > 2) {
                i_ind.push_back(k);
            }
    }


    vector<Point2f> left_point_f; //Oringinal left lane points for transform to BEV
    vector<Point2f> right_point_f; // Oringinal right lane points for transform to BEV
    vector<Point2f> extra_right_point_f;
    vector<Point2f> warp_left_point_f; // Transformed BEV left lane points
    vector<Point2f> warp_right_point_f; // Transformes BEV right lane points;   
    vector<Point2f> warp_extra_right_point_f;    
          
    for(int k = 0; k < OUTPUT_H-1; k++) {
        for(int ll = 0; ll < i_ind.size(); ll++) {
            if(expect[OUTPUT_W * k + i_ind[ll]] > 0 ) {
                if ( int( vis_h * tusimple_row_anchor[OUTPUT_H - 1 - k] / INPUT_H) - 1 > 460) {
                        break;
                }
                Point temp_point;  
                if ( ll == 0 && expect[OUTPUT_W * k + i_ind[ll]] != 1) {  // left
                    temp_point.x = int(expect[OUTPUT_W * k + i_ind[ll]] * col_sample_w * vis_w / INPUT_W) - 1;
                    temp_point.y = int( vis_h * tusimple_row_anchor[OUTPUT_H - 1 - k] / INPUT_H) - 1;
                    left_point_f.push_back(temp_point);
                }
                else if ( ll == 1 && expect[OUTPUT_W * k + i_ind[ll]] != 1) { // right
                    temp_point.x = int(expect[OUTPUT_W * k + i_ind[ll]] * col_sample_w * vis_w / INPUT_W) - 1;
                    temp_point.y = int( vis_h * tusimple_row_anchor[OUTPUT_H - 1 - k] / INPUT_H) - 1;
                    right_point_f.push_back(temp_point);   
                }
                else if ( ll == 2 && expect[OUTPUT_W * k + i_ind[ll]] != 1) { // extra_right
                    temp_point.x = int(expect[OUTPUT_W * k + i_ind[ll]] * col_sample_w * vis_w / INPUT_W) - 1;
                    temp_point.y = int( vis_h * tusimple_row_anchor[OUTPUT_H - 1 - k] / INPUT_H) - 1;
                    extra_right_point_f.push_back(temp_point);   
                }
            }
        }
    }
    if (left_point_f.size() != 0 && right_point_f.size() != 0 && extra_right_point_f.size() != 0) { 
    perspectiveTransform(left_point_f , warp_left_point_f, trans); // Transform left lane points to BEV left lane points
    perspectiveTransform(right_point_f , warp_right_point_f, trans); // Transform right lane points to BEV right lane points
    perspectiveTransform(extra_right_point_f , warp_extra_right_point_f, trans); // Transform right lane points to BEV right lane points
    vector<int> left_point_f_y; // Y BEV left lane points to get polyfit
    vector<int> left_point_f_x; // X BEV left lane points to get polyfit
    vector<int> right_point_f_y; // Y BEV right lane points to get polyfit
    vector<int> right_point_f_x; // X BEV right lane points to get polyfit
    vector<int> extra_right_point_f_x;
    vector<int> extra_right_point_f_y;
    vector<int> orig_left_point_f_y;
    vector<int> orig_left_point_f_x;
    vector<int> orig_right_point_f_y;
    vector<int> orig_right_point_f_x;
    vector<int> orig_extra_right_point_f_y;
    vector<int> orig_extra_right_point_f_x;

    for (int i =0 ; i < warp_left_point_f.size(); i++) {
        if (warp_left_point_f[i].y < distance_ && option_) break; // Dynamic ROI
            Point pp =
                { warp_left_point_f[i].x,
                warp_left_point_f[i].y };
              cv::circle(warped_frame, pp, 4, CV_RGB(0, 255 ,0), -1);
              left_point_f_y.push_back(warp_left_point_f[i].y);
              left_point_f_x.push_back(warp_left_point_f[i].x);         
        }

    for (int i =0 ; i < warp_right_point_f.size() ; i++) {
        if (warp_right_point_f[i].y < distance_ && option_) break; // Dynamic ROI
            Point pp =
                    { warp_right_point_f[i].x,
                      warp_right_point_f[i].y };
        cv::circle(warped_frame, pp, 4, CV_RGB(0, 255 ,0), -1);
        right_point_f_y.push_back(warp_right_point_f[i].y);
        right_point_f_x.push_back(warp_right_point_f[i].x);
    }

    for (int i =0 ; i < warp_extra_right_point_f.size() ; i++) {
        if (warp_extra_right_point_f[i].y < distance_ && option_) break; // Dynamic ROI
            Point pp =
                    { warp_extra_right_point_f[i].x,
                      warp_extra_right_point_f[i].y };
        cv::circle(warped_frame, pp, 4, CV_RGB(0, 255 ,0), -1);
        extra_right_point_f_y.push_back(warp_extra_right_point_f[i].y);
        extra_right_point_f_x.push_back(warp_extra_right_point_f[i].x);
    }
    for (int i =0 ; i < left_point_f.size() ; i++) {
        orig_left_point_f_y.push_back(left_point_f[i].y);
        orig_left_point_f_x.push_back(left_point_f[i].x);
    }
    for (int i =0 ; i < right_point_f.size() ; i++) {
        orig_right_point_f_y.push_back(right_point_f[i].y);
        orig_right_point_f_x.push_back(right_point_f[i].x);
    }
    for (int i =0 ; i < extra_right_point_f.size() ; i++) {
        orig_extra_right_point_f_y.push_back(extra_right_point_f[i].y);
        orig_extra_right_point_f_x.push_back(extra_right_point_f[i].x);
    }



    // To get center polyfit
    left_coef_ = polyfit(left_point_f_y, left_point_f_x);
    right_coef_ = polyfit(right_point_f_y, right_point_f_x);
    extra_right_coef_ = polyfit(extra_right_point_f_y, extra_right_point_f_x);
    orig_left_coef_ = polyfit(orig_left_point_f_y, orig_left_point_f_x);
    orig_right_coef_ = polyfit(orig_right_point_f_y, orig_right_point_f_x);
    orig_extra_right_coef_ = polyfit(orig_extra_right_point_f_y, orig_extra_right_point_f_x);
    center_coef_ = (left_coef_ + right_coef_)/2;    
    extra_center_coef_ = (right_coef_ + extra_right_coef_)/2;
    get_lane_coef();


  if (_view) {
        vector<Point2f> warp_left_point2f;
        vector<Point2f> warp_right_point2f;
        vector<Point2f> warp_extra_right_point2f;
        vector<Point2f> reverse_left_point2f;
        vector<Point2f> reverse_right_point2f;
        vector<Point2f> reverse_extra_right_point2f;


       //To draw lane in BEV
        vector<Point> left_point;
        vector<Point> right_point;
        vector<Point> extra_right_point;
        vector<Point> center_point;
        vector<Point> extra_center_point;
            for (int i = option_ ? distance_: 0; i <= 480; i++) {
                Point temp_left_point;
                Point temp_right_point;
                Point temp_center_point;
                Point temp_extra_right_point;
                Point temp_extra_center_point;

                temp_left_point.x = (int)((left_coef_.at<float>(2, 0) * pow(i, 2)) + (left_coef_.at<float>(1, 0) * i) + left_coef_.at<float>(0, 0));
                temp_left_point.y = (int)i;
                temp_right_point.x = (int)((right_coef_.at<float>(2, 0) * pow(i, 2)) + (right_coef_.at<float>(1, 0) * i) + right_coef_.at<float>(0, 0));
                temp_right_point.y = (int)i;
                temp_center_point.x = (int)((center_coef_.at<float>(2, 0) * pow(i, 2)) + (center_coef_.at<float>(1, 0) * i) + center_coef_.at<float>(0, 0));
                temp_center_point.y = (int)i;
                temp_extra_center_point.x = (int)((extra_center_coef_.at<float>(2, 0) * pow(i, 2)) + (extra_center_coef_.at<float>(1, 0) * i) + extra_center_coef_.at<float>(0, 0));
                temp_extra_center_point.y = (int)i;
                temp_extra_right_point.x = (int)((extra_right_coef_.at<float>(2, 0) * pow(i, 2)) + (extra_right_coef_.at<float>(1, 0) * i) + extra_right_coef_.at<float>(0, 0));
                temp_extra_right_point.y = (int)i;

                left_point.push_back(temp_left_point);
                right_point.push_back(temp_right_point);
                center_point.push_back(temp_center_point);
                extra_center_point.push_back(temp_extra_center_point);
                extra_right_point.push_back(temp_extra_right_point);

                warp_left_point2f.push_back(temp_left_point);
                warp_right_point2f.push_back(temp_right_point);
                warp_extra_right_point2f.push_back(temp_extra_right_point);
            }


            const Point* left_points_point_ = (const cv::Point*) Mat(left_point).data;
            int left_points_number_ = Mat(left_point).rows;
            const Point* right_points_point_ = (const cv::Point*) Mat(right_point).data;
            int right_points_number_ = Mat(right_point).rows;
            const Point* center_points_point_ = (const cv::Point*) Mat(center_point).data;
            int center_points_number_ = Mat(center_point).rows;
            const Point* extra_center_points_point_ = (const cv::Point*) Mat(extra_center_point).data;
            int extra_center_points_number_ = Mat(extra_center_point).rows;
            const Point* extra_right_points_point_ = (const cv::Point*) Mat(extra_right_point).data;
            int extra_right_points_number_ = Mat(extra_right_point).rows;


            polylines(warped_frame, &left_points_point_, &left_points_number_, 1, false, Scalar(255, 100, 100), 10);
            polylines(warped_frame, &right_points_point_, &right_points_number_, 1, false, Scalar(255, 100, 100), 10);
            polylines(warped_frame, &center_points_point_, &center_points_number_, 1, false, Scalar(200, 255, 200), 10);
            polylines(warped_frame, &extra_center_points_point_, &extra_center_points_number_, 1, false, Scalar(200, 255, 200), 10);
            polylines(warped_frame, &extra_right_points_point_, &extra_right_points_number_, 1, false, Scalar(255, 100, 100), 10);

            //To draw lane in Original image
            perspectiveTransform(warp_left_point2f , reverse_left_point2f, reverse_trans);
            perspectiveTransform(warp_right_point2f , reverse_right_point2f, reverse_trans);
            perspectiveTransform(warp_extra_right_point2f , reverse_extra_right_point2f, reverse_trans);
            
            std::vector<Point> leftInt, rightInt, extrarightInt;
            for(const auto& p:reverse_left_point2f) leftInt.push_back(Point(static_cast<int>(p.x), static_cast<int>(p.y)));
            for(const auto& p:reverse_right_point2f) rightInt.push_back(Point(static_cast<int>(p.x), static_cast<int>(p.y)));
            for(const auto& p:reverse_extra_right_point2f) extrarightInt.push_back(Point(static_cast<int>(p.x), static_cast<int>(p.y)));

            const Point* leftInt_points = (const cv::Point*) Mat(leftInt).data;
            int leftInt_points_number_ = Mat(leftInt).rows;
            const Point* rightInt_points = (const cv::Point*) Mat(rightInt).data;
            int rightInt_points_number_ = Mat(rightInt).rows;
            const Point* extrarightInt_points = (const cv::Point*) Mat(extrarightInt).data;
            int extrarightInt_points_number_ = Mat(extrarightInt).rows;

            polylines(_frame, &leftInt_points, &leftInt_points_number_, 1, false, Scalar(255, 100, 100), 10);
            polylines(_frame, &rightInt_points, &rightInt_points_number_, 1, false, Scalar(255, 100, 100), 10);          
            polylines(_frame, &extrarightInt_points, &extrarightInt_points_number_, 1, false, Scalar(255, 100, 100), 10);

            //fillPoly, Drivable Area
            //std::reverse(rightInt.begin(), rightInt.end());
            //leftInt.insert(leftInt.end(), rightInt.begin(), rightInt.end());
            //std::vector<std::vector<cv::Point>> pts;
            //pts.push_back(leftInt);

            //fillPoly(orig_img, pts, cv::Scalar(255,191,0));

            //addWeighted( orig_img, 0.3, _frame, 0.7,0.0,blended);
            //end fillpoly


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
    if(!warped_frame.empty()) {
      imshow("Window2", warped_frame);
    }
    if(!_frame.empty()){
      imshow("Window3", _frame);
    }

    waitKey(_delay);
  }
  clear_release();
    }
}

} /* namespace lane_detect */
