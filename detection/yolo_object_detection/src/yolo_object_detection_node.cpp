#include "yolo_object_detection_node.hpp"

namespace yolo_object_detection_ros2
{

YoloObjectDetectionNode::YoloObjectDetectionNode()
    : Node("yolo_object_detection_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true))
{
  if (!readParameters()){
    rclcpp::shutdown();
  }

  init();
}

YoloObjectDetectionNode::~YoloObjectDetectionNode()
{
  delete yoloDetector_;
}

bool YoloObjectDetectionNode::readParameters(){
  std::string names_file, cfg_file, weights_file;
  std::string names_path, cfg_path, weights_path;

  // Path to names file
  this->get_parameter_or("yolo_model/names_file/name", names_file, std::string("obj.names"));
  this->get_parameter_or("names_path", names_path, std::string("/default"));
  names_ = names_path + "/" + names_file;

  // Path to cfg file
  this->get_parameter_or("yolo_model/cfg_file/name", cfg_file, std::string("yolov4-custom_best.cfg"));
  this->get_parameter_or("cfg_path", cfg_path, std::string("/default"));
  cfg_ = cfg_path + "/" + cfg_file;

  // Path to weights file
  this->get_parameter_or("yolo_model/weights_file/name", weights_file, std::string("yolov3-tiny-scale_truck.weights"));
  this->get_parameter_or("weights_path", weights_path, std::string("/default"));
  weights_ = weights_path + "/" + weights_file;

  // Image parameters
  this->get_parameter_or("image/width", width_, 640);
  this->get_parameter_or("image/height", height_, 480);

  // Load common parameters
  this->get_parameter_or("image_view/enable_opencv", viewImage_, false);
  this->get_parameter_or("image_view/wait_key_delay", waitKeyDelay_, 1);
  this->get_parameter_or("image_view/enable_console_output", enableConsoleOutput_, false);

  return true;
}

void YoloObjectDetectionNode::init() {
  RCLCPP_INFO(this->get_logger(),"Yolo_Init Start");
  //gettimeofday(&startTime_, NULL);

  // Initialize publisher and subscriber
  // std::string rearCamTopicName;
  // int rearCamQueueSize;
  std::string frontCamTopicName;
  int frontCamQueueSize;
  std::string runYoloTopicName;
  int runYoloQueueSize;

  std::string BboxTopicName;
  int BboxQueueSize;

  // tmp ///
  std::string BboxArrayTopicName;
  int BboxArrayQueueSize = 10;
  ////////////////////

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  // this->get_parameter_or("subscribers/rear_camera_reading/topic", rearCamTopicName, std::string("rear_cam/image_raw"));
  // this->get_parameter_or("subscribers/rear_camera_reading/queue_size", rearCamQueueSize, 1);
  this->get_parameter_or("subscribers/front_camera_reading/topic", frontCamTopicName, std::string("usb_cam/image_raw"));
  this->get_parameter_or("subscribers/front_camera_reading/queue_size", frontCamQueueSize, 1);
  this->get_parameter_or("subscribers/run_yolo_/topic", runYoloTopicName, std::string("run_yolo_flag"));
  this->get_parameter_or("subscribers/run_yolo_/queue_size", runYoloQueueSize, 1);
  
  /****************************/
  /* Ros Topic Publish Option */
  /****************************/
  // this->get_parameter_or("publishers/Bbox/topic", BboxTopicName, std::string("/yolo_object_detection/Bbox"));
  // this->get_parameter_or("publishers/Bbox/queue_size", BboxQueueSize, 1);


  // tmp//
  this->get_parameter_or("publishers/BboxArray/topic", BboxArrayTopicName, std::string("yolo_object_detection/BboxArray"));
  this->get_parameter_or("publishers/BboxArray/queue_size", BboxArrayQueueSize, 1);
  //////

  /************************/
  /* Ros Topic Subscriber */
  /************************/
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  frontCamImgSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(frontCamTopicName, qos, std::bind(&YoloObjectDetectionNode::frontCamImgCallback, this, std::placeholders::_1));

  // rearCamImgSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>(rearCamTopicName, rearCamQueueSize, std::bind(&YoloObjectDetectionNode::rearCamImgCallback, this, std::placeholders::_1));

  runYoloSubscriber_ = this->create_subscription<ros2_msg::msg::Yoloflag>(runYoloTopicName, runYoloQueueSize, std::bind(&YoloObjectDetectionNode::runYoloCallback, this, std::placeholders::_1)); 

  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  // BboxPublisher_ = this->create_publisher<ros2_msg::msg::Bbox>(BboxTopicName, BboxQueueSize);
  

  BboxArrayPublisher_ = this->create_publisher<ros2_msg::msg::BboxArray>(BboxArrayTopicName, qos);
  CurImagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>("cur_image",qos);
  
  /***************/
  /* Start Setup */
  /***************/
  cv::Mat img_for_init(width_, height_, CV_8UC3, cv::Scalar(0,255,0)); 

  objectNames_ = objectNames(names_); // 미리 해두는건 어떤지?

  yoloDetector_ = new Detector(cfg_, weights_, 0.95f/* thresh*/);
  yoloDetector_->detect(img_for_init);
  yoloDetector_->detect(img_for_init);
  yoloDetector_->detect(img_for_init);
  std::cerr << " init finish" << std::endl;
  detectThread_ = std::thread(&YoloObjectDetectionNode::detectInThread, this);
}

std::vector<std::string> YoloObjectDetectionNode::objectNames(std::string const filename)
{
  std::ifstream file(filename);
  std::vector<std::string> file_lines;
  if (!file.is_open()) return file_lines;
  for(std::string line; getline(file, line);) file_lines.push_back(line);
  std::cout << "object names loaded\n";

  return file_lines;
}

void YoloObjectDetectionNode::runYoloCallback(const ros2_msg::msg::Yoloflag::SharedPtr msg){
  f_run_yolo_ = msg->f_run_yolo;
  // r_run_yolo_ = msg->r_run_yolo;
}

// void YoloObjectDetectionNode::rearCamImgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
// {
//   if (r_run_yolo_){
//     cv_bridge::CvImagePtr cv_ptr;
  
//     try {
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception& e) {
//       RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
//       return;
//     }
  
//     if (cv_ptr) {
//       std::scoped_lock lock(rear_cam_mutex_);
//       rearCamImageCopy_ = cv_ptr->image.clone();
//       resize(rearCamImageCopy_, rearCamImageCopy_, cv::Size(width_, height_));
//       sec_ = msg->header.stamp.sec;
//       nsec_ = msg->header.stamp.nanosec;
//       rearImageStatus_ = true;
//     }
//   }
// }

void YoloObjectDetectionNode::frontCamImgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{ 
  if (f_run_yolo_) {
    cv_bridge::CvImagePtr cv_ptr = NULL;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),"cv_bridge exception: %s", e.what());
      return;
    }
  
    if (cv_ptr) {
      std::scoped_lock lock(front_cam_mutex_);
      frontCamImageCopy_ = cv_ptr->image.clone();
      cur_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      resize(frontCamImageCopy_, frontCamImageCopy_, cv::Size(width_, height_));
      ImageStatus_ = true;
    }
  }
}

// Detector -> 기존의 yolo class

void YoloObjectDetectionNode::detectInThread()
{
  RCLCPP_INFO(this->get_logger(),"detectInThread Start");
  struct timeval endTime;
  static double time = 0.0;
  //  static int cnt = 0;
  uint8_t mark = 0;
  while (rclcpp::ok()) {
    objects_.clear(); // 보류
    processing_objects.clear();
    if (ImageStatus_) {
       std::scoped_lock lock(front_cam_mutex_);
      // cv::Mat mat1(height_, width_, CV_8UC3, cv::Scalar(255, 255, 255));
      objects_ = yoloDetector_->detect(frontCamImageCopy_);
      //processing_objects = removeDuplication(objects_);
      // objects_ = yoloDetector_->detect(mat1);
      //if(objects_.size() == 0 ) //std::cerr  << " empty "  << std::endl;

      
       cv::Mat draw_img;

       draw_img = frontCamImageCopy_.clone();
      // cv::imshow("YOLO", mat1);
       if (!draw_img.empty()) {
         drawBoxes(draw_img, objects_);
       }
       cv::imshow("YOLO", draw_img);
       cv::waitKey(waitKeyDelay_);
      
      sensor_msgs::msg::Image::SharedPtr output_msg =  cur_image_->toImageMsg();
      CurImagePublisher_->publish(*output_msg);
      publishInThread(objects_);
      ImageStatus_ = false;
    }


    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

// 해당부분 수정
void YoloObjectDetectionNode::publishInThread(std::vector<bbox_t> objects)
{
  ros2_msg::msg::Bbox box;
  ros2_msg::msg::BboxArray boxes; 
  boxes.header.stamp = this->now();
  boxes.header.frame_id="boxes";
  // unsigned int max_bbox_size = 0;
  // x, y --> start point
  // Find max size bbox
  
  unsigned int class_id = 1;
  for (auto &i : objects) {
      // if (i.obj_id == 2) {
      //   cnt0 += 1;
      //   break;
      // }
      box.class_id = i.obj_id;
      box.x = i.x + i.w / 2;
      box.y = i.y + i.h / 2;
      box.w = i.w;
      box.h = i.h;
      boxes.boxes.push_back(box);
    }

  // box.class_id = bbox.obj_id;
  // RCLCPP_INFO(this->get_logger(),"obj_id : %d",bbox.obj_id);
  //RCLCPP_INFO(this->get_logger(),"obj_box : [%d, %d, %d, %d], size : %d", box.x, box.y, box.w, box.h, objects.size());
  
  // BboxPublisher_->publish(msg);
  BboxArrayPublisher_->publish(boxes);
}

std::vector<bbox_t> YoloObjectDetectionNode::removeDuplication(std::vector<bbox_t> objects)
{
  std::vector<bbox_t> processing_objects;
  int size = objects.size();
  int cx, cy, comp_x, comp_y;
  bool is_check[size] = {false};
  for (int i = 0; i < size; i++) {
    if (is_check[i]) {
      continue;
    }
    bbox_t comp_box;
    deque<int> dq;
    dq.push_back(i);
    is_check[i] = true;
    comp_box.obj_id = objects.at(i).obj_id;
    comp_box.x = objects.at(i).x;
    comp_box.y = objects.at(i).y;
    comp_box.w = objects.at(i).w;
    comp_box.h = objects.at(i).h;
    comp_box.prob = objects.at(i).prob;
    while (!dq.empty()) {
      int now = dq.at(0);
      dq.pop_front();
      cx = objects.at(now).x + objects.at(now).w / 2;
      cy = objects.at(now).y + objects.at(now).h / 2;
      for (int j = 0; j < size; j++) {
        if (is_check[j]) {
          continue;
        }
        else {
          comp_x = objects.at(j).x + objects.at(j).w / 2;
          comp_y = objects.at(j).y + objects.at(j).h / 2;

          if (CheckDuplication(cx, cy, comp_x, comp_y, 10)) {
            is_check[j] = true;
            if (comp_box.prob < objects[j].prob) {
              dq.push_back(j);
              comp_box.obj_id = objects.at(j).obj_id;
              comp_box.x = objects.at(j).x;
              comp_box.y = objects.at(j).y;
              comp_box.w = objects.at(j).w;
              comp_box.h = objects.at(j).h;
              comp_box.prob = objects.at(j).prob;
            }
            else continue;
          }
          else continue;
        }
      }
    }
    processing_objects.push_back(comp_box);
  }
  return processing_objects;
}

bool YoloObjectDetectionNode::CheckDuplication(int std_x, int std_y, int comp_x, int comp_y, int std_num)
{
  int check_num = std::pow(comp_x - std_x, 2) + std::pow(comp_y - std_y, 2);
  if (check_num <= std::pow(std_num, 2)) return true;;
  return false;
}



void YoloObjectDetectionNode::drawBoxes(cv::Mat mat_img, std::vector<bbox_t> objects)
{
  int const colors[6][3] = { { 1,0,1 },{ 0,0,1 },{ 0,1,1 },{ 0,1,0 },{ 1,1,0 },{ 1,0,0 } };
  int cnt = 0;
  for(auto &i : objects)
  {
    cv::Scalar color = obj_id_to_color(i.obj_id);
    cv::rectangle(mat_img, cv::Rect(i.x, i.y, i.w, i.h), color, 2);
    cnt += 1;
    // if(objectNames_.size() > i.obj_id)
    // {
    //   std::string obj_name = objectNames_[i.obj_id];
    //   if (i.track_id > 0) obj_name += " - " + std::to_string(i.track_id);
    //   cv::Size const text_size = getTextSize(obj_name, cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, 2, 0);
    //   int max_width = (text_size.width > i.w + 2) ? text_size.width : (i.w + 2);
    //   max_width = std::max(max_width, (int)i.w + 2);

    //   cv::rectangle(mat_img, cv::Point2f(std::max((int)i.x - 1, 0), std::max((int)i.y - 35, 0)),
    //                 cv::Point2f(std::min((int)i.x + max_width, mat_img.cols - 1), std::min((int)i.y, mat_img.rows - 1)), color, CV_FILLED, 8, 0);
    //   putText(mat_img, obj_name, cv::Point2f(i.x, i.y - 16), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
    //   cnt += 1;
    // }
    if (cnt > 5) break;
  }
  cv::imshow("YOLO", mat_img);
  cv::waitKey(waitKeyDelay_);
  
}

void YoloObjectDetectionNode::recordData(struct timeval startTime){
  struct timeval currentTime;
  char file_name[] = "YOLO_log00.csv";
  static char file[128] = {0x00, };
  char buf[256] = {0x00,};
  static bool flag = false;
  double diff_time;
  std::ifstream read_file;
  std::ofstream write_file;
  std::string log_path = "/home/logfiles/";
  if(!flag){
    for(int i = 0; i < 100; i++){
      file_name[8] = i/10 + '0';  //ASCII
      file_name[9] = i%10 + '0';
      sprintf(file, "%s%s", log_path.c_str(), file_name);
      read_file.open(file);
      if(read_file.fail()){  //Check if the file exists
        read_file.close();
        write_file.open(file);
        break;
      }
      read_file.close();
    }
    write_file << "time,lvReqtoYoloDelay" << std::endl; //seconds, miliseconds
    flag = true;
  }
  else{
    //gettimeofday(&currentTime, NULL);
    diff_time = ((currentTime.tv_sec - startTime.tv_sec)) + ((currentTime.tv_usec - startTime.tv_usec)/1000000.0);
    {
      std::scoped_lock lock(rear_cam_mutex_);
      sprintf(buf, "%.10e, %.10e", diff_time, delay_);
    }
    write_file.open(file, std::ios::out | std::ios::app);
    write_file << buf << std::endl;
  }
  write_file.close();
}

} // namespace yolo_object_detection
