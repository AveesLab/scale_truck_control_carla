#include "v2v.hpp"

namespace v2v {

V2V::V2V()
       : Node("v2v_node", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true)) {
    this->get_parameter("truck_name", truck_name);
    size_t pos = truck_name.find("truck");
    if (pos != std::string::npos) {
        try {
            pos += 5;  
            number = std::stoi(truck_name.substr(pos));
            number++;
            to_ =  "truck" + std::to_string(number);
        } catch (const std::invalid_argument& e) {
            std::cerr << "Error: No valid number found in the string" << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "Error: Number out of range" << std::endl;
        }
    }

    if(truck_name == "truck0") TargetSubscriber_ = this->create_subscription<ros2_msg::msg::Target>("target",10,std::bind(&V2V::TargetSubCallback,this,std::placeholders::_1));
    V2VPublisher_ =  this->create_publisher<ros2_msg::msg::Target>("/v2v/" + to_, 10);
    V2VSubscriber_ = this->create_subscription<ros2_msg::msg::Target>("/v2v/" + truck_name,10,std::bind(&V2V::V2VSubCallback,this,std::placeholders::_1));
    if(truck_name != "truck0" ) TargetPublisher_ = this->create_publisher<ros2_msg::msg::Target>("target" , 10);
    DistanceSubscriber_ = this->create_subscription<ros2_msg::msg::Obj2xav>("min_distance", 10, std::bind(&V2V::DistanceSubCallback, this, std::placeholders::_1));
    VelocitySubscriber = this->create_subscription<std_msgs::msg::Float32>("velocity",1,std::bind(&V2V::velocity_callback, this, std::placeholders::_1));
//    InfoPublisher_=this->create_publisher<std_msgs::msg::Float32MultiArray>("DrivingInfo",100);
    rclcpp::QoS CmdPubQos(10);
    CmdPubQos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    InfoPublisher_=this->create_publisher<ros2_msg::msg::Xav2cmd>("DrivingInfo", CmdPubQos);
}

void V2V::TargetSubCallback(const ros2_msg::msg::Target::SharedPtr msg) {
    this->emergency_flag = msg->emergency_flag;
    this->target_velocity = msg->tar_vel;
    this->target_distance = msg->tar_dist;

    ros2_msg::msg::Target tar;
    tar.emergency_flag = this->emergency_flag;
    tar.tar_vel = this->target_velocity;
    tar.tar_dist = this->target_distance;
    V2VPublisher_->publish(tar);
} 

void V2V::V2VSubCallback(const ros2_msg::msg::Target::SharedPtr msg) {
    this->emergency_flag = msg->emergency_flag;
    this->target_velocity = msg->tar_vel;
    this->target_distance = msg->tar_dist;

    ros2_msg::msg::Target tar;
    tar.emergency_flag = this->emergency_flag;
    tar.tar_vel = this->target_velocity;
    tar.tar_dist = this->target_distance;
    V2VPublisher_->publish(tar);
    TargetPublisher_->publish(tar);
}
void V2V::DistanceSubCallback(const ros2_msg::msg::Obj2xav::SharedPtr msg) {
    this->current_distance = msg->min_dist;
}
void V2V::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    cur_vel_ = msg->data; // update current velocity
   
//    std_msgs::msg::Float32MultiArray info;
//    info.data.resize(0);
//    info.data.push_back(number);
//    info.data.push_back(current_distance);
//    info.data.push_back(cur_vel_);
    ros2_msg::msg::Xav2cmd info;
    info.src_index = this->number - 1;
    info.cur_dist = this->current_distance;
    info.cur_vel = this->cur_vel_;
    InfoPublisher_->publish(info);
}

}
