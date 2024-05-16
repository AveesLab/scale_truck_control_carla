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
  this->get_parameter_or("subscribers/lane_to_xavier/topic", LaneTopicName, std::string("laneinfo"));
  this->get_parameter_or("subscribers/lane_to_xavier/queue_size", LaneQueueSize, 1);
  
  /************************/
  /* Ros Topic Subscriber */
  /************************/
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
  qos.best_effort();
  XavSubscriber_ = this->create_subscription<ros2_msg::msg::Xav2lane>(XavSubTopicName, XavSubQueueSize, std::bind(&LaneKeeping::XavSubCallback, this, std::placeholders::_1));
  LaneSubscriber_ = this->create_subscription<ros2_msg::msg::Lane2xav>(LaneTopicName, LaneQueueSize, std::bind(&LaneKeeping::LaneSubCallback, this, std::placeholders::_1));

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

  K1_ = (a_[0] * pow(1.2f, 4)) + (a_[1] * pow(1.2f, 3)) + (a_[2] * pow(1.2f, 2)) + (a_[3] * 1.2f) + a_[4];
  K2_ = (b_[0] * pow(1.2f, 4)) + (b_[1] * pow(1.2f, 3)) + (b_[2] * pow(1.2f, 2)) + (b_[3] * 1.2f) + b_[4];
}
//sub cur vel from speed control
void LaneKeeping::XavSubCallback(const ros2_msg::msg::Xav2lane::SharedPtr msg)
{
  cur_vel_ = msg->cur_vel;
  get_steer_coef(cur_vel_);
}

//sub coef from lane detection
void LaneKeeping::LaneSubCallback(const ros2_msg::msg::Lane2xav::SharedPtr msg) 
{ 
  poly_coef_.coef = msg->coef;
  left.at<float>(2,0) = poly_coef_.coef[0].a;
  left.at<float>(1,0) = poly_coef_.coef[0].b;
  left.at<float>(0,0) = poly_coef_.coef[0].c;

  right.at<float>(2,0) = poly_coef_.coef[1].a;
  right.at<float>(1,0) = poly_coef_.coef[1].b;
  right.at<float>(0,0) = poly_coef_.coef[1].c;

  center.at<float>(2,0) = poly_coef_.coef[2].a;
  center.at<float>(1,0) = poly_coef_.coef[2].b;
  center.at<float>(0,0) = poly_coef_.coef[2].c;

  controlSteer(left, right, center);
  SteerPublisher_->publish(steer_);
  
}

void LaneKeeping::get_steer_coef(float vel){
  float value;
  if (vel > 1.2f)
    value = 1.2f;
  else
    value = vel;

  if (value < 0.65f){
    K1_ = K2_ =  K_;
  }
  else{
    K1_ = (a_[0] * pow(value, 4)) + (a_[1] * pow(value, 3)) + (a_[2] * pow(value, 2)) + (a_[3] * value) + a_[4];
    K2_ = (b_[0] * pow(value, 4)) + (b_[1] * pow(value, 3)) + (b_[2] * pow(value, 2)) + (b_[3] * value) + b_[4];
  }
  
}

void LaneKeeping::controlSteer(Mat left, Mat right, Mat center) {
  float car_position = 320.0; //width of image / 2
  float l1 = 0.0f, l2 = 0.0f;
  float i = 480.0 * eL_height_;  
  float j = 480.0 * trust_height_;

  if (!left.empty() && !right.empty()) {

    lane_coef_.coef[0].a = left.at<float>(2, 0);
    lane_coef_.coef[0].b = left.at<float>(1, 0);
    lane_coef_.coef[0].c = left.at<float>(0, 0);

    lane_coef_.coef[1].a = right.at<float>(2, 0);
    lane_coef_.coef[1].b = right.at<float>(1, 0);
    lane_coef_.coef[1].c = right.at<float>(0, 0);

    lane_coef_.coef[2].a = center.at<float>(2, 0);
    lane_coef_.coef[2].b = center.at<float>(1, 0);
    lane_coef_.coef[2].c = center.at<float>(0, 0);

    l1 =  j - i;
    l2 = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - ((lane_coef_.coef[2].a * pow(j, 2)) + (lane_coef_.coef[2].b * j) + lane_coef_.coef[2].c);

    e_values_[0] = ((lane_coef_.coef[2].a * pow(i, 2)) + (lane_coef_.coef[2].b * i) + lane_coef_.coef[2].c) - car_position;  //eL
    e_values_[1] = e_values_[0] - (lp_ * (l2 / l1));  //e1

    SteerAngle_ = ((-1.0f * K1_) * e_values_[1]) + ((-1.0f * K2_) * e_values_[0]);
    steer_.data = SteerAngle_;
//    cout << SteerAngle_  << '\n';
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

