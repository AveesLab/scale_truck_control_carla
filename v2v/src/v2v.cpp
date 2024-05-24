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
            int number = std::stoi(truck_name.substr(pos));
            number++;
            to_ =  "truck" + std::to_string(number);
        } catch (const std::invalid_argument& e) {
            std::cerr << "Error: No valid number found in the string" << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "Error: Number out of range" << std::endl;
        }
    }
    timer_ = this->create_wall_timer(100ms, std::bind(&V2V::timerCallback, this));
    if(truck_name == "truck0") TargetSubscriber_ = this->create_subscription<ros2_msg::msg::Target>("target",10,std::bind(&V2V::TargetSubCallback,this,std::placeholders::_1));
    V2VPublisher_ =  this->create_publisher<ros2_msg::msg::Target>("/v2v/" + to_, 10);
    V2VSubscriber_ = this->create_subscription<ros2_msg::msg::Target>("/v2v/" + truck_name,10,std::bind(&V2V::V2VSubCallback,this,std::placeholders::_1));
    VelocitySubscriber = this->create_subscription<std_msgs::msg::Float32>("velocity",1,std::bind(&V2V::velocity_callback, this, std::placeholders::_1));
    EmergencySubscriber = this->create_subscription<std_msgs::msg::Bool>("emergency_brake",10,std::bind(&V2V::emergencyCallback,this,std::placeholders::_1));
    if(truck_name != "truck0" ) TargetPublisher_ = this->create_publisher<ros2_msg::msg::Target>("target" , 10);
    
}

void V2V::TargetSubCallback(const ros2_msg::msg::Target::SharedPtr msg) {
    this->emergency_flag = msg->emergency_flag;
    this->target_distance = msg->tar_dist;
} 

void V2V::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    this->current_velocity = msg->data;
}

void V2V::emergencyCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->emergency_flag = msg->data;
}

void V2V::timerCallback() {
    ros2_msg::msg::Target tar;
    tar.emergency_flag = this->emergency_flag;
    tar.tar_vel = this->current_velocity;
    tar.tar_dist = this->target_distance;
    V2VPublisher_->publish(tar);
}

void V2V::V2VSubCallback(const ros2_msg::msg::Target::SharedPtr msg) {
    ros2_msg::msg::Target tar;
    tar.emergency_flag = msg->emergency_flag;
    tar.tar_vel = msg->tar_vel;
    tar.tar_dist = msg->tar_dist;
    this->emergency_flag  = msg->emergency_flag;
    this->target_distance = msg->tar_dist;
    TargetPublisher_->publish(tar);
}


}