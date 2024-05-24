#include "planner.hpp"

namespace Planner {



Planner::Planner()
       : Node("Planner", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
    this->get_parameter("truck_name", truck_name);
    if (truck_name == "truck0") lv = true;
    DistanceSubscriber_ = this->create_subscription<std_msgs::msg::Float32>("min_distance", 10, std::bind(&Planner::DistanceSubCallback, this, std::placeholders::_1));
    TargetSubscriber_ = this->create_subscription<ros2_msg::msg::Target>("target",10,std::bind(&Planner::TargetSubCallback,this,std::placeholders::_1));
    EmergencyPublisher_ = this->create_publisher<std_msgs::msg::Bool>("emergency_brake", 10);
    VelocitySubscriber_ = this->create_subscription<std_msgs::msg::Float32>("velocity",1,std::bind(&Planner::velocity_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(10ms, std::bind(&Planner::timerCallback, this));
    TargetVelocityPublisher_ = this->create_publisher<std_msgs::msg::Float32>("target_velocity", 10);
}

void Planner::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    this->current_velocity = msg->data;
}

void Planner::TargetSubCallback(const ros2_msg::msg::Target::SharedPtr msg) {
    this->target_velocity = msg->tar_vel;
    this->target_distance = msg->tar_dist;
    this->emergency_flag = msg->emergency_flag;
}

void Planner::DistanceSubCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    this->current_distance = msg->data;
}

bool Planner::check_collision() {
    if(current_distance > 0.0f && current_distance <= 13.0f) return true;
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

void Planner::calculate_cacc_param() 
{
    des_spacing = this->target_distance *  this->current_velocity;
    spacing_err = this->current_distance - des_spacing;
    speed_err = this->target_velocity - this->current_velocity;
}

float Planner::calculate_target_velocity_cacc() {
    float t_speed = 0.0f;
    t_speed = current_velocity + myGapControlGainGap * spacing_err + myGapControlGainGapDot * speed_err;
    return t_speed;
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
        calculate_cacc_param();
        msg.data = calculate_target_velocity_cacc();
        TargetVelocityPublisher_->publish(msg);
    }

} 


}