#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_msg/msg/ocr2lrc.hpp"
#include "ros2_msg/msg/lrc2ocr.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
class Controller : public rclcpp::Node
{
public:
    Controller();
    int Index_;
    float tx_vel_ = 0.0;
    float tx_dist_;
    float tx_tdist_;
    float cur_vel_ = 0.0;
    float est_vel_;
    float preceding_truck_vel_;
    float output_;
    float ref_vel = -1.0f;
    bool emergency_stop_ = false;
    
    std_msgs::msg::Float32 control_msg_;
    float Kp_dist_ = 0.8; // 2.0; //0.8;
    float Kd_dist_ = 0.03; //0.05;
    float Kp_throttle_ = 2.5; // 2.0; //0.8;
    float Ki_throttle_ = 0.5; // 0.4; //10.0;
    float Ka_throttle_ = 0.01;
    float Kf_throttle_ = 1.0;  // feed forward const.
    float Kp_brake_;
    float Ki_brake_;
    float Ka_brake_;
    float Kf_brake_;
    //float dt_ = 0.1;

    bool sync_ = false;
    bool sync_with_delay = false;
    bool target_velocity_ = false;
    bool no_wait_ = false;
    bool CurVelArrived = false;

private:
    void TargetVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void WaitVelSubCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    void SetSpeed();
    void Throttle_PID(double dt_, float tar_vel, float current_vel);
    void Brake_PID(double dt_, float tar_vel, float current_vel);
    void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ControlPublisher;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr TargetVelocitysubscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr VelocitySubscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr WaitVelSubscriber_;
};

//float tar_vel, float current_vel
