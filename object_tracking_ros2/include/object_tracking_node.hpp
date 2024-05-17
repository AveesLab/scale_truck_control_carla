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
#include "ros2_msg/msg/bbox_array.hpp"
// #include "src/tracker.h"
// #include "ros2_msg/msg/??" // Maybe Distance?

using namespace std;
using namespace std::chrono_literals;


typedef struct box {
    float x, y, w, h;
} box;


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
    // void RadarCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

    // Tracker ObjectTracker_;
    /* ROS2 sub */
    rclcpp::Subscription<ros2_msg::msg::BboxArray>::SharedPtr BboxArraySubscriber_;
    /* pub */

    vector<box> detected_objects;
    std::thread trackThread_;

    bool radarStatus_ = false;
    struct timeval startTime_;
};
}

#endif