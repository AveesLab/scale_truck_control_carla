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


using namespace std::chrono_literals;
using std::placeholders::_1;
class CACC_sumo : public rclcpp::Node
{
public:
    CACC_sumo();
    int Index_;
    float tx_vel_ = 0.0;
    float tx_dist_;
    float tx_tdist_;
    float cur_vel_ = 0.0;
    float est_vel_;
    float preceding_truck_vel_;
    float output_;

    bool emergency_stop_ = false;
    
    float des_spacing = 0.0f;
    float spacing_err = 0.0f;
    float speed_err = 0.0f;
    float time_gap = 0.5f;
    float front_dis = 0.0f;
    float front_v = 0.0f;
    float accel = 0.0f;
    float target_speed = 0.0f;
    float myGapClosingControlGainGap = 0.45f;
    float myGapClosingControlGainGapDot = 0.3f;
    float myCollisionAvoidanceGainGap = 0.45f;
    float myCollisionAvoidanceGainGapDot = 0.3f;
    float myGapControlGainGap = 0.45f;
    float myGapControlGainGapDot = 0.3f;

private:
    void LrcCallback(const ros2_msg::msg::Lrc2ocr::SharedPtr msg);
    void velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void accel_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void shutdown_callback(const std_msgs::msg::String::SharedPtr msg);
    void calc_param();
    float calc_speed(); 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ros2_msg::msg::Lrc2ocr>::SharedPtr publisher_;
    rclcpp::Subscription<ros2_msg::msg::Lrc2ocr>::SharedPtr subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr VelocitySubscriber;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr AccelSubscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ShutdownSubscriber;
};

//float tar_vel, float current_vel
