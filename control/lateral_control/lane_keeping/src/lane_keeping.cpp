#include "lane_keeping.hpp"

namespace lanekeeping {

LaneKeeping::LaneKeeping()
       : Node("LaneKeeping", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
  /**************/
  /* ROS2 Topic */
  /**************/
  std::string XavSubTopicName;
  int XavSubQueueSize;
  std::string LaneTopicName;
  int LaneQueueSize; 
  std::string XavPubTopicName;
  int XavPubQueueSize;

  /******************************/
  /* Ros Topic Subscribe Option */
  /******************************/
  this->get_parameter_or("subscribers/xavier_to_lane/topic", XavSubTopicName, std::string("xav2lane_msg"));
  this->get_parameter_or("subscribers/xavier_to_lane/queue_size", XavSubQueueSize, 1);
  this->get_parameter_or("subscribers/lane_to_xavier/topic", LaneTopicName, std::string("lane2xav_msg"));
  this->get_parameter_or("subscribers/lane_to_xavier/queue_size", LaneQueueSize, 1);
  
  /************************/
  /* Ros Topic Subscriber */
  /************************/
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  qos.best_effort();
  XavSubscriber_ = this->create_subscription<ros2_msg::msg::Xav2lane>(XavSubTopicName, XavSubQueueSize, std::bind(&LaneKeeping::XavSubCallback, this, std::placeholders::_1));
  LaneSubscriber_ = this->create_subscription<ros2_msg::msg::Lane2xav>(LaneTopicName, LaneQueueSize, std::bind(&LaneKeeping::LaneSubCallback, this, std::placeholders::_1));
  VelSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("velocity", 1, std::bind(&LaneKeeping::VelSubCallback, this, std::placeholders::_1));
  LaneChangeSubscriber_ = this->create_subscription<std_msgs::msg::Int32>("lane_change", 1, std::bind(&LaneKeeping::LaneChangeSubCallback, this, std::placeholders::_1));
  /***********************/
  /* Ros Topic Publisher */
  /***********************/
  SteerPublisher_ = this->create_publisher<std_msgs::msg::Float32>("steer_control",XavPubQueueSize);

  /********** PID control ***********/
//  left_coef_ = Mat::zeros(3, 1, CV_32F);
//  right_coef_ = Mat::zeros(3, 1, CV_32F);
//  center_coef_ = Mat::zeros(3, 1, CV_32F);

    lane_coef_.coef.resize(3);
    poly_coef_.coef.resize(3);
    e_values_.resize(2);
    left = Mat::zeros(3, 1, CV_32F);
    right = Mat::zeros(3, 1, CV_32F);
    center = Mat::zeros(3, 1, CV_32F);
    for (int i = 0; i < 8; i++) {
        line_.emplace_back(Mat::zeros(3, 1, CV_32F));
    }
  /* Lateral Control coefficient */
  this->get_parameter_or("params/K", K_, 0.15f);

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

  LoadParams();
}

void LaneKeeping::LoadParams(void)
{
  this->get_parameter_or("LaneKeeping/eL_height",eL_height_, 0.2f);  
  this->get_parameter_or("LaneKeeping/e1_height",e1_height_, 1.1852f);  
  this->get_parameter_or("LaneKeeping/lp",lp_, 609.3f);  
  this->get_parameter_or("LaneKeeping/steer_angle",SteerAngle_, 0.0f);
  this->get_parameter_or("LaneKeeping/trust_height",trust_height_, 0.6667f);  
  value_k1 = 1.2f;
  value_k3 = 1.32f; // 1.3
  K1_ = (a_[0] * pow(value_k1, 4)) + (a_[1] * pow(value_k1, 3)) + (a_[2] * pow(value_k1, 2)) + (a_[3] * value_k1) + a_[4];
  K2_ = (b_[0] * pow(value_k1, 4)) + (b_[1] * pow(value_k1, 3)) + (b_[2] * pow(value_k1, 2)) + (b_[3] * value_k1) + b_[4];
  K3_ = (a_[0] * pow(value_k3, 4)) + (a_[1] * pow(value_k3, 3)) + (a_[2] * pow(value_k3, 2)) + (a_[3] * value_k3) + a_[4];
  K4_ = (b_[0] * pow(value_k3, 4)) + (b_[1] * pow(value_k3, 3)) + (b_[2] * pow(value_k3, 2)) + (b_[3] * value_k3) + b_[4];
}
//sub cur vel from speed control
void LaneKeeping::XavSubCallback(const ros2_msg::msg::Xav2lane::SharedPtr msg)
{
  cur_vel_ = msg->cur_vel;
  get_steer_coef(cur_vel_);
}

//sub cur vel from speed control
void LaneKeeping::VelSubCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  cur_vel_ = msg->data;;
  get_steer_coef(cur_vel_);
}


//sub coef from lane detection
void LaneKeeping::LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg) 
{ 
  poly_coef_.coef = msg->coef;
  line_[0].at<float>(2,0) = poly_coef_.coef[0].a; // lane 0
  line_[0].at<float>(1,0) = poly_coef_.coef[0].b;
  line_[0].at<float>(0,0) = poly_coef_.coef[0].c;

  line_[1].at<float>(2,0) = poly_coef_.coef[1].a; // lane 1
  line_[1].at<float>(1,0) = poly_coef_.coef[1].b;
  line_[1].at<float>(0,0) = poly_coef_.coef[1].c;

  line_[2].at<float>(2,0) = poly_coef_.coef[2].a;  //center_0_1
  line_[2].at<float>(1,0) = poly_coef_.coef[2].b;
  line_[2].at<float>(0,0) = poly_coef_.coef[2].c;

  line_[3].at<float>(2,0) = poly_coef_.coef[3].a; //lane 2
  line_[3].at<float>(1,0) = poly_coef_.coef[3].b;
  line_[3].at<float>(0,0) = poly_coef_.coef[3].c;

  line_[4].at<float>(2,0) = poly_coef_.coef[4].a; // cenrter_1_2
  line_[4].at<float>(1,0) = poly_coef_.coef[4].b;
  line_[4].at<float>(0,0) = poly_coef_.coef[4].c;

  if(line_[1].at<float>(0,0) > 320.0) current_center= 2;
  else current_center=4;
  controlSteer();

  SteerPublisher_->publish(steer_);
  
}

void LaneKeeping::LaneChangeSubCallback(const std_msgs::msg::Int32::SharedPtr msg) {

    if(msg->data == 2) {
      if(current_center == 2) {
      lane_keeping = false;
      lc_right_flag_ = true;
      }
    }
    else if(msg->data == 4) {
      if(current_center == 4) {
      lane_keeping = false;
      lc_left_flag_ = true;
      }
    }

}

tk::spline LaneKeeping::cspline(int mark_) {
  float tY1_ = ((float)480) * 0;
  float tY2_ = ((float)480) * 0.1;
  float tY3_ = ((float)480) * 0 +80.0f;
  float tY4_ = ((float)480) * 0.01 + 80.0f;
  float tY5_ = ((float)480) * e1_height_;
  
  std::vector<double> X;
  std::vector<double> Y; 
  tk::spline cspline_eq;

  /*****************/
  /* lc_right_coef */
  /*****************/
  if (mark_ == 1 &&  !line_[4].empty()) { 
    int tX3_ = (int)((line_[4].at<float>(2, 0) * pow(tY3_, 2)) + (line_[4].at<float>(1, 0) * tY3_) + line_[4].at<float>(0, 0));
    int tX4_ = (int)((line_[4].at<float>(2, 0) * pow(tY4_, 2)) + (line_[4].at<float>(1, 0) * tY4_) + line_[4].at<float>(0, 0));
    X = {(double)tX3_, (double)tX4_, (double)(640/2)};
    Y = {(double)tY3_, (double)tY4_, (double)480}; 


    tk::spline s(Y, X, tk::spline::cspline); 
    cspline_eq = s;
  }
  /****************/
  /* lc_left_coef */
  /****************/
  else if (mark_ == 2 && !line_[2].empty()) { 

    int tX3_ = (int)((line_[2].at<float>(2, 0) * pow(tY3_, 2)) + (line_[2].at<float>(1, 0) * tY3_) + line_[2].at<float>(0, 0));
    int tX4_ = (int)((line_[2].at<float>(2, 0) * pow(tY4_, 2)) + (line_[2].at<float>(1, 0) * tY4_) + line_[2].at<float>(0, 0));
  
    X = {(double)tX3_, (double)tX4_, (double)(640/2)};
    Y = {(double)tY3_, (double)tY4_, (double)480}; 
  
    tk::spline s(Y, X, tk::spline::cspline); 
    cspline_eq = s;
  }

  return cspline_eq;
}

void LaneKeeping::get_steer_coef(float vel){
  float value;
  if (vel > 8.0f) {
    value_k1 = 1.2f;
    value_k3 = 1.35f;
  }
  else if(vel < 8.0f) {
    value_k1 = 0.6f;
    value_k3 = 0.005f;
  }


  K1_ = (a_[0] * pow(value_k1, 4)) + (a_[1] * pow(value_k1, 3)) + (a_[2] * pow(value_k1, 2)) + (a_[3] * value_k1) + a_[4];
  K2_ = (b_[0] * pow(value_k1, 4)) + (b_[1] * pow(value_k1, 3)) + (b_[2] * pow(value_k1, 2)) + (b_[3] * value_k1) + b_[4];
  K3_ = (a_[0] * pow(value_k3, 4)) + (a_[1] * pow(value_k3, 3)) + (a_[2] * pow(value_k3, 2)) + (a_[3] * value_k3) + a_[4];
  K4_ = (b_[0] * pow(value_k3, 4)) + (b_[1] * pow(value_k3, 3)) + (b_[2] * pow(value_k3, 2)) + (b_[3] * value_k3) + b_[4];
  
}

void LaneKeeping::controlSteer() {
  float height_ = 640.0f;
  float car_position = 320.0; //width of image / 2
  float l1 = 0.0f, l2 = 0.0f,l3=0.0f;
  float i = 480.0 * eL_height_;  
  float j = 480.0 * trust_height_;
  float k = 480.0 * e1_height_;

  std::cerr << "current_center  "  << current_center << std::endl;
  if (!left.empty() && !right.empty() && lane_keeping) {
    lane_coef_.coef[2].a = line_[current_center].at<float>(2, 0);
    lane_coef_.coef[2].b = line_[current_center].at<float>(1, 0);
    lane_coef_.coef[2].c = line_[current_center].at<float>(0, 0);

    l1 =  j - i;
    l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);

    e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //e1

    SteerAngle_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
    steer_.data = SteerAngle_;
   // std::cerr << "keepg angle:  "  << SteerAngle_ << std::endl;
//    cout << SteerAngle_  << '\n';
    return;
  } 

  if ((!line_[4].empty()) && lc_right_flag_ ) {
    lane_coef_.coef[2].a = line_[4].at<float>(2, 0);
    lane_coef_.coef[2].b = line_[4].at<float>(1, 0);
    lane_coef_.coef[2].c = line_[4].at<float>(0, 0);

    int mark_ = 1;
    tk::spline cspline_eq_ = cspline(mark_);
    l1 =  j - i;
    l3 = (float)cspline_eq_(i) - (float)cspline_eq_(j);
    e_values_[0] = (float)cspline_eq_(i) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l3 / l1));  //trust_e1
    e_values_[2] = (float)cspline_eq_(k)- car_position;  //e1
    SteerAngle_ = ((-1.0f * K3_) * e_values_[1]) + ((-1.0f * K4_) * e_values_[0]);
    //std::cerr << "change to right angle:  "  << SteerAngle_ << std::endl;
    steer_.data = SteerAngle_;
    float temp_diff = ((lane_coef_.coef[2].a * pow(height_, 2)) + (lane_coef_.coef[2].b * height_) + lane_coef_.coef[2].c) - 320; 
     std::cerr << "current_center changes: "  << temp_diff << std::endl;
    if(temp_diff <= 7.0){
      //right_cnt++;
      //if(right_cnt >20){
        lc_right_flag_ = false;
        lane_keeping = true;
        right_cnt=0;
        current_center = 4; 
     //}
    }
    return;
  }
  else if ((!line_[2].empty()) && lc_left_flag_ ) {
    lane_coef_.coef[2].a = line_[2].at<float>(2, 0);
    lane_coef_.coef[2].b = line_[2].at<float>(1, 0);
    lane_coef_.coef[2].c = line_[2].at<float>(0, 0);

    int mark_ = 2;
    tk::spline cspline_eq_ = cspline(mark_);
    l1 =  j - i;
    l3 = (float)cspline_eq_(i) - (float)cspline_eq_(j);
    e_values_[0] = (float)cspline_eq_(i) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l3 / l1));  //trust_e1
    e_values_[2] = (float)cspline_eq_(k)- car_position;  //e1
    SteerAngle_ = ((-1.0f * K3_) * e_values_[1]) + ((-1.0f * K4_) * e_values_[0]);
    steer_.data = SteerAngle_;
    float temp_diff = ((lane_coef_.coef[2].a * pow(height_, 2)) + (lane_coef_.coef[2].b * height_) + lane_coef_.coef[2].c) - 320; 

    if(temp_diff >= -1.0 ){
     // left_cnt++;
     // if(left_cnt >30){
        lc_left_flag_ = false;
        lane_keeping = true;
        left_cnt=0;
        current_center = 2; 
        std::cerr << "current_center left changes: "  << temp_diff << std::endl;
    //  }
    }
    return;
  }
  else {
    lane_coef_.coef[2].a = line_[current_center].at<float>(2, 0);
    lane_coef_.coef[2].b = line_[current_center].at<float>(1, 0);
    lane_coef_.coef[2].c = line_[current_center].at<float>(0, 0);

    l1 =  j - i;
    l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);

    e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //e1

    SteerAngle_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
    steer_.data = SteerAngle_;
   // std::cerr << "keepg angle:  "  << SteerAngle_ << std::endl;
//    cout << SteerAngle_  << '\n';
    return;
  }
 }

} /* namespace lane_detect */


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<lanekeeping::LaneKeeping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

