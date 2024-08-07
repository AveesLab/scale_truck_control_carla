#include "speed_control/speed_control_node.hpp"


Controller::Controller()
          : Node("controller", rclcpp::NodeOptions()
                              .allow_undeclared_parameters(true)
                              .automatically_declare_parameters_from_overrides(true))
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    this->get_parameter_or("carla_sync",sync_,false);
    this->get_parameter_or("carla_sync_with_delay",sync_with_delay,false);

    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    
    TargetVelocitysubscriber_ = this->create_subscription<std_msgs::msg::Float32>(
      "target_velocity", 
      qos_profile,
      std::bind(&Controller::TargetVelocityCallback, this, _1)
    );
  
    VelocitySubscriber = this->create_subscription<std_msgs::msg::Float32>("velocity",1,std::bind(&Controller::velocity_callback, this, _1));
    ControlPublisher = this->create_publisher<std_msgs::msg::Float32>("velocity_control",1);
    WaitVelSubscriber_ = this->create_subscription<std_msgs::msg::Bool>("wait_vel",qos,std::bind(&Controller::WaitVelSubCallback,this,_1));
    timer_ = this->create_wall_timer(
        10ms, std::bind(&Controller::SetSpeed, this));
}   

void Controller::velocity_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    CurVelArrived = true;
    cur_vel_ = msg->data; // update current velocity
}

void Controller::WaitVelSubCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if(msg->data) no_wait_ = false;
    else no_wait_ = true;
}

void Controller::TargetVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    target_velocity_ = true;
    tx_vel_ = msg->data;
}


void Controller::SetSpeed() {
    if (sync_) {
        if(target_velocity_ && CurVelArrived) {
          target_velocity_ = false;
          CurVelArrived = false;
        }
        else {
          return;
        }
    }
    else if (sync_with_delay) {
        if(target_velocity_ && CurVelArrived) {
            target_velocity_ = false;
            CurVelArrived = false;
        }
        else if (no_wait_) {
            if(CurVelArrived) {
                no_wait_ = false;
                CurVelArrived = false;
            }
        }
        else {
            return;
        }
    }
    float dt_ = 0.01;
    Throttle_PID(dt_, tx_vel_, cur_vel_);
}

void Controller::Throttle_PID(double dt_, float tar_vel, float current_vel)
{
    static float err, P_err, I_err;
    static float prev_u_b, prev_u_k, prev_u, A_err;
    static float dist_err, prev_dist_err, P_dist_err, D_dist_err;
    float u = 0.f, u_k = 0.f, u_b = 0.f;
    float u_dist = 0.f, u_dist_k = 0.f;

    
    if(tar_vel >= 0) {
        err = tar_vel - current_vel;
        P_err = Kp_throttle_ * err;
        I_err += Ki_throttle_ * err * dt_;
        A_err += Ka_throttle_ * ((prev_u_k - prev_u) / dt_);

      
        u = P_err + I_err + A_err + tar_vel * Kf_throttle_;

        

        if(u >= 1.0) u_k = 1.0f;
        else if(u <= -1.0) u_k = -1.0f;
        else u_k = u;

        control_msg_.data = u_k;
        ControlPublisher->publish(control_msg_);
        prev_u_k = u_k;
        prev_u = u;
    }
    else {
        P_err = 0;
        I_err = 0;
        A_err = 0;
        u_k = -1.0f;
        control_msg_.data = u_k;
        ControlPublisher->publish(control_msg_);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
