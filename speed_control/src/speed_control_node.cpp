#include "speed_control/speed_control_node.hpp"


Controller::Controller()
          : Node("controller")
{
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

  subscriber_ = this->create_subscription<ros2_msg::msg::Lrc2ocr>(
    "lrc2ocr_msg", 
    qos_profile,
    std::bind(&Controller::LrcCallback, this, _1)
  );
  publisher_ = this->create_publisher<ros2_msg::msg::Ocr2lrc>("topic", qos_profile);
}   


void Controller::LrcCallback(const ros2_msg::msg::Lrc2ocr::SharedPtr msg) 
{
  Index_ = msg->index;
  tx_dist_ = msg->cur_dist;
  tx_tdist_ = msg->tar_dist;
  tx_vel_ = msg->tar_vel;
  cur_vel_ = msg->cur_vel;

  SetSpeed();
  publisher_->publish(pub_msg_);
}


void Controller::SetSpeed()
{
  std::chrono::high_resolution_clock::time_point cur_time = std::chrono::high_resolution_clock::now();
  static std::chrono::high_resolution_clock::time_point prev_time;
  double dt_ = std::chrono::duration_cast<std::chrono::microseconds>(cur_time - prev_time).count() / 1000000;
  Throttle_PID(dt_, tx_vel_, cur_vel_);
  prev_time = cur_time;
}

void Controller::Throttle_PID(double dt_, float tar_vel, float current_vel)
{
  static float err, P_err, I_err;
  static float prev_u_b, prev_u_k, prev_u, A_err;
  static float dist_err, prev_dist_err, P_dist_err, D_dist_err;
  float u = 0.f, u_k = 0.f, u_b = 0.f;
  float u_dist = 0.f, u_dist_k = 0.f;
  float ref_vel = 0.f;
  pub_msg_.cur_vel = current_vel;

  if(Index_ != 10) {
    dist_err = tx_dist_ - tx_tdist_;    
    P_dist_err = Kp_dist_ * dist_err;
    D_dist_err = (Kd_dist_ * ((dist_err - prev_dist_err) / dt_ )); 
    u_dist = P_dist_err + D_dist_err + tar_vel;

    // sat(u(k))  saturation start 
    if(u_dist > 1.2) u_dist_k = 1.2;
    else if(u_dist <= 1.2 && u_dist >= -1.2) u_dist_k = u_dist;
    else u_dist_k = -1.2;

    ref_vel = u_dist_k;
  } else {
    ref_vel = tar_vel;
  }

  pub_msg_.ref_vel = ref_vel;
  
  if(ref_vel >= 0)
  {
    err = ref_vel - current_vel;
    P_err = Kp_throttle_ * err;
    I_err += Ki_throttle_ * err * dt_;
    A_err += Ka_throttle_ * ((prev_u_k - prev_u) / dt_);

    if(tar_vel <= 0){
      P_err = 0;
      I_err = 0;
      A_err = 0;
    }
  
    u = P_err + I_err + A_err + ref_vel * Kf_throttle_;

    if(u > 2.0) u_k = 2.0;
    else if(u <= 0) u_k = 0;
    else u_k = u;

    pub_msg_.u_k = u_k;
  
    prev_u_k = u_k;
    prev_u = u;
    prev_dist_err = dist_err;
  }
  else
  {
    err = ref_vel - current_vel;
    P_err = Kp_brake_ * err;
    I_err += Ki_brake_ * err * dt_;
    A_err += Ka_brake_ * ((prev_u_b - prev_u) / dt_);

    if(tar_vel <= 0){
      P_err = 0;
      I_err = 0;
      A_err = 0;
    }
  
    u = P_err + I_err + A_err + ref_vel * Kf_brake_;

    if(u > -2.0) u_b = -2.0;
    else if(u <= 0) u_b = 0;
    else u_b = u;
  
    pub_msg_.u_b = u_b;
    prev_u_b = u_b;
    prev_u = u;
    prev_dist_err = dist_err;

  }
}

int main(int argc, char * argv[])
{
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<Controller>());
rclcpp::shutdown();
return 0;
}




//float tar_vel, float current_vel
//, std::placeholders::_1, std::placeholders::_2