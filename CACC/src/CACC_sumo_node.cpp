#include "CACC_sumo_node.hpp"

CACC_sumo::CACC_sumo()
          : Node("CACC_sumo")
{
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

  subscriber_ = this->create_subscription<ros2_msg::msg::Lrc2ocr>(
    "lrc2ocr_msg", 
    qos_profile,
    std::bind(&CACC_sumo::LrcCallback, this, _1)
  );
  VelocitySubscriber = this->create_subscription<std_msgs::msg::Float32>("velocity_info",1,std::bind(&CACC_sumo::velocity_callback, this, _1));
  AccelSubscriber = this->create_subscription<std_msgs::msg::Float32>("accel_info",1,std::bind(&CACC_sumo::velocity_callback, this, _1));
  publisher_ = this->create_publisher<ros2_msg::msg::Lrc2ocr>("plan2ctr_msg", qos_profile);
}   


void CACC_sumo::calc_param() 
{
    des_spacing = this->time_gap *  cur_vel_;
    spacing_err = this->front_dis - des_spacing;
    speed_err = this->front_v - cur_vel_;
}

float CACC_sumo::calc_speed() 
{
    float t_speed = 0.0f;
    
    if(abs(spacing_err) <= 1.0) { // speed mode
        t_speed = cur_vel_ + myGapControlGainGap * spacing_err + myGapControlGainGapDot * speed_err;
        //RCLCPP_INFO(this->get_logger(), "Speed Mode, spaing_err : %f, speed_err : %f",spacing_err,speed_err);
        return t_speed;
    }
    else if(spacing_err < -3.0) { //collision avoidance mode
        t_speed = cur_vel_ + myCollisionAvoidanceGainGap * spacing_err + myCollisionAvoidanceGainGapDot * speed_err;
        //RCLCPP_INFO(this->get_logger(), "collision avoidance mode, spaing_err : %f, speed_err : %f",spacing_err,speed_err);
        return t_speed;
    }
    else { // Gap closing mode
        t_speed = cur_vel_ + myGapClosingControlGainGap * spacing_err + myGapClosingControlGainGapDot*speed_err;
        //RCLCPP_INFO(this->get_logger(), " Gap closing mode, spaing_err : %f, speed_err : %f",spacing_err,speed_err);
        return t_speed;
    }

}
//export LD_LIBRARY_PATH=/home/nvidia/ros2_ws/install/lane_detection_ros2/lib/lane_detection_ros2:$LD_LIBRARY_PATH

void CACC_sumo::LrcCallback(const ros2_msg::msg::Lrc2ocr::SharedPtr msg) 
{
    Index_ = msg->index;
    front_dis = msg->cur_dist;
    time_gap = msg->tar_dist; //time_gap;
    front_v = msg->tar_vel;
    emergency_stop_ = msg->emergency_flag;
    if(Index_ != 10) {
        calc_param();

        target_speed = calc_speed();

        ros2_msg::msg::Lrc2ocr ocr;

        ocr.index = msg->index;
        ocr.steer_angle = msg->steer_angle;
        ocr.cur_dist = msg->cur_dist;
        ocr.tar_dist =  msg->tar_dist;
        ocr.tar_vel = target_speed;
        ocr.est_vel = msg->est_vel;
        ocr.emergency_flag = msg->emergency_flag;
        publisher_->publish(ocr);
    }
    else {
        ros2_msg::msg::Lrc2ocr ocr;
        ocr.index = msg->index;
        ocr.steer_angle = msg->steer_angle;
        ocr.cur_dist = msg->cur_dist;
        ocr.tar_dist =  msg->tar_dist;
        ocr.tar_vel = msg->tar_vel;
        ocr.est_vel = msg->est_vel;
        ocr.emergency_flag = msg->emergency_flag;
        publisher_->publish(ocr);      
    }
}


void CACC_sumo::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    cur_vel_ = msg->data; // update current velocity
}

void CACC_sumo::accel_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    accel = msg->data; // update current accleration
}


int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<CACC_sumo>());
rclcpp::shutdown();
return 0;
}