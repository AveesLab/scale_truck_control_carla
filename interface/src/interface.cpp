#include "interface.hpp"

namespace interface {

interface::interface()
       : Node("interface_node", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true)) {
    /**************/
    /* ROS2 Topic */
    /**************/
    //subscribe : {target distance, target velocity, emergency flag} from controller
    //publish   : {target distance, target velocity, emergency flag} to LV

    rclcpp::QoS CmdSubQos(10);
    CmdSubQos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    CmdSubQos.durability(rclcpp::DurabilityPolicy::Volatile);

    CmdSubscriber_ = this->create_subscription<ros2_msg::msg::Cmd2xav>("/cmd2xav_msg",CmdSubQos,std::bind(&interface::CmdSubCallback,this,std::placeholders::_1));

    TargetPublisher_ =  this->create_publisher<ros2_msg::msg::Target>("target", 10);
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

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<interface::interface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
