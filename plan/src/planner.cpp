#include "planner.hpp"

namespace Planner {



Planner::Planner()
       : Node("Planner", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
    this->get_parameter("truck_name", truck_name);
    if (truck_name == "truck0") lv = true;
    DistanceSubscriber_ = this->create_subscription<ros2_msg::msg::Obj2xav>("min_distance", 10, std::bind(&Planner::DistanceSubCallback, this, std::placeholders::_1));
    TargetSubscriber_ = this->create_subscription<ros2_msg::msg::Target>("target",10,std::bind(&Planner::TargetSubCallback,this,std::placeholders::_1));
    EmergencyPublisher_ = this->create_publisher<std_msgs::msg::Bool>("emergency_brake", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&Planner::timerCallback, this));
    TargetVelocityPublisher_ = this->create_publisher<std_msgs::msg::Float32>("target_velocity", 10);
}

void Planner::TargetSubCallback(const ros2_msg::msg::Target::SharedPtr msg) {
    this->target_velocity = msg->tar_vel;
    this->target_distance = msg->tar_dist;
    this->emergency_flag = msg->emergency_flag;
}

void Planner::DistanceSubCallback(const ros2_msg::msg::Obj2xav::SharedPtr msg) {
    this->current_distance = msg->min_dist;
}

bool Planner::check_collision() {
    if(current_distance > 0.0f && current_distance <= 3.0f) return true;
    else return false;
}

void Planner::send_full_brake() {
    std_msgs::msg::Float32 msg;
    msg.data = -1.0f;
    TargetVelocityPublisher_->publish(msg);
}

float Planner::calculate_target_velocity() {
    static float dist_err, prev_dist_err, P_dist_err, D_dist_err;
    float u_dist = 0.f, u_dist_k = 0.f;
    float ref_vel = 0.f;

    dist_err = this->current_distance - this->target_distance;
    P_dist_err = this->Kp_dist_ * dist_err;
    D_dist_err = (this->Kd_dist_ * ((dist_err - prev_dist_err) / this->dt_ ));
    u_dist = P_dist_err + D_dist_err + this->target_velocity;

    // sat(u(k))  saturation start 
    if(u_dist > 90.0) u_dist_k = 90.0;
    else if(u_dist <= 0) u_dist_k = 0;
    else u_dist_k = u_dist;

    ref_vel = u_dist_k;
    prev_dist_err = dist_err;
    return ref_vel;
}

void Planner::timerCallback() {
    if( ( this->emergency_flag  )|| (!lv && this->emergency_brake)) {
        send_full_brake();
        return;
    }
    
    if(lv && check_collision()) {
        std_msgs::msg::Bool msg;
        msg.data = true;
        EmergencyPublisher_->publish(msg);
        send_full_brake();
        return;
    }
    else {
        std_msgs::msg::Bool msg;
        msg.data = false;
        EmergencyPublisher_->publish(msg);
    }

    if(lv) {
        std_msgs::msg::Float32 msg;
        msg.data = this->target_velocity;
        TargetVelocityPublisher_->publish(msg);
    }
    else {
        std_msgs::msg::Float32 msg;
        msg.data = calculate_target_velocity();
        TargetVelocityPublisher_->publish(msg);
    }

} 


}