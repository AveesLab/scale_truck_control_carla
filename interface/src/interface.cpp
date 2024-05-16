#include "interface.hpp"

namespace interface {

interface::interface()
       : Node("interface_node", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true)) {
    
    rclcpp::QoS CmdSubQos(10);
    CmdSubQos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    CmdSubQos.durability(rclcpp::DurabilityPolicy::Volatile);

    rclcpp::QoS ScenarioSubQos(10);
    CmdSubQos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    CmdSubQos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    CmdSubscriber_ = this->create_subscription<ros2_msg::msg::Cmd2xav>("/cmd2xav_msg",CmdSubQos,std::bind(&interface::CmdSubCallback,this,std::placeholders::_1));

    TargetPublisher_ =  this->create_publisher<ros2_msg::msg::Target>("target", 10);

    ScenarioVelocitySubscriber_ = this->create_subscription<std_msgs::msg::Float32>("/cmd2xav_target_vel",ScenarioSubQos,std::bind(&interface::ScenarioVelocitySubCallback,this,std::placeholders::_1));
    ScenarioBrakeSubscriber_ = this->create_subscription<std_msgs::msg::Bool>("/cmd2xav_brake",ScenarioSubQos,std::bind(&interface::ScenarioBrakeSubCallback,this,std::placeholders::_1));
    ScenarioTimegapSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("/cmd2xav_timegap",ScenarioSubQos,std::bind(&interface::ScenarioTimegapSubCallback,this,std::placeholders::_1));
}

void interface::ScenarioVelocitySubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    this->target_velocity = msg->data;

    ros2_msg::msg::Target tar;
    tar.emergency_flag = this->emergency_flag;
    tar.tar_vel = this->target_velocity;
    tar.tar_dist = this->target_distance;
    TargetPublisher_->publish(tar);

}

void interface::ScenarioBrakeSubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    this->emergency_flag = msg->data;
    ros2_msg::msg::Target tar;
    tar.emergency_flag = this->emergency_flag;
    tar.tar_vel = this->target_velocity;
    tar.tar_dist = this->target_distance;
    TargetPublisher_->publish(tar);
}
void interface::ScenarioTimegapSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    this->target_distance = msg->data;
    ros2_msg::msg::Target tar;
    tar.emergency_flag = this->emergency_flag;
    tar.tar_vel = this->target_velocity;
    tar.tar_dist = this->target_distance;
    TargetPublisher_->publish(tar);
}


void interface::CmdSubCallback(const ros2_msg::msg::Cmd2xav::SharedPtr msg) {
    this->emergency_flag = msg->emergency_flag;
    this->target_velocity = msg->tar_vel;
    this->target_distance = msg->tar_dist;

    ros2_msg::msg::Target tar;
    tar.emergency_flag = this->emergency_flag;
    tar.tar_vel = this->target_velocity;
    tar.tar_dist = this->target_distance;
    TargetPublisher_->publish(tar);
}


}