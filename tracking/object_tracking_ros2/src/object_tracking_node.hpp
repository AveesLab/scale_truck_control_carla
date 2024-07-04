#ifndef OBJECT_TRACKING_NODE_HPP
#define OBJECT_TRACKING_NODE_HPP

//C++
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <thread>
#include <mutex>
#include <string>


//ROS2
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/fusing.hpp"
#include "ros2_msg/msg/bbox_array.hpp"
#include "ros2_msg/msg/fusing_array.hpp"
#include "ros2_msg/msg/tracking_array.hpp"
#include "type.h"
#include "tracker.h"
using namespace std;
using namespace std::chrono_literals;




namespace object_tracking_ros2 {

class ObjectTrackingNode : public rclcpp::Node
{
public:
    explicit ObjectTrackingNode();
    ~ObjectTrackingNode();
private:
    void init();
    void TrackingCallback();
    void publishInThread();
    void trackInThread();
    void BboxArrayCallback(const ros2_msg::msg::BboxArray::SharedPtr msg);
    void FusingArrayCallback(const ros2_msg::msg::FusingArray::SharedPtr msg);
    std::mutex fusing_objects_mutex_;
    Tracker ObjectTracker_ = Tracker();
    // rclcpp::Subscription<ros2_msg::msg::BboxArray>::SharedPtr BboxArraySubscriber_;
    rclcpp::Subscription<ros2_msg::msg::FusingArray>::SharedPtr FusingArraySubscriber_;
    rclcpp::Publisher<ros2_msg::msg::TrackingArray>::SharedPtr TrackingPublisher_;
    vector<box> detected_objects;

    vector<object_info> fusing_objects;
    
    std::thread trackThread_;

    bool radarStatus_ = false;
    struct timeval startTime_;
};
}

#endif