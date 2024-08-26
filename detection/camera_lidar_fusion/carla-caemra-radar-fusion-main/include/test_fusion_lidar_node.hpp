//C++
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <thread>
#include <mutex>
#include <string>

#include <opencv2/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>



//ROS2
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "ros2_msg/msg/bbox_array.hpp"
#include "ros2_msg/msg/fusing_array.hpp"
#include "ros2_msg/msg/fusing.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

namespace test_fusion {

class test_fusion_node : public rclcpp::Node
{
public:
    explicit test_fusion_node();
    ~test_fusion_node();

private:
    rclcpp::Publisher<ros2_msg::msg::FusingArray>::SharedPtr FusingPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void callback(const ros2_msg::msg::BboxArray::ConstSharedPtr image_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg);
    void timerCallback();

    rclcpp::Subscription<ros2_msg::msg::BboxArray>::SharedPtr BboxArraySubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr PointcloudSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ImageSubscriber_;
    ros2_msg::msg::BboxArray bbox_msg;
    sensor_msgs::msg::PointCloud2 points_msg;
    void BboxArraySubCallback(const ros2_msg::msg::BboxArray::SharedPtr msg);
    void PointcloudSubCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void ImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    bool isDataReady();

    cv::Mat frontCamImageCopy_;
    cv::Mat image_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat transform_;

    bool isBboxReady = false;
    bool isPointsReady = false;
    bool isImageReady = false;
};

} // namespace yolo_object_detectio