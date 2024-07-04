#ifndef SENSOR_FUSING_NODE_HPP
#define SENSOR_FUSING_NODE_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <thread>
#include <mutex>
#include <string>
#include <vector>
#include <map>
#include <cmath>

#include <opencv2/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

// PCL //

#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rclcpp/rclcpp.hpp"
#include "ros2_msg/msg/bbox_array.hpp"
#include "ros2_msg/msg/fusing_array.hpp"
#include "ros2_msg/msg/fusing.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono_literals;

// typedef struct box {
//     float x, y, w, h;
//     int class_id;
// } box;

typedef struct radar_info {
    float x, y, z, range, velocity, azimuth;
} radar_info;

typedef struct fusing_info {
    float x, y, w, h, distance, velocity;
    int class_id;
} fusing_info;

typedef struct points {
    float x, y, z;
} points;

namespace sensor_fusing_ros2 {

class SensorFusingNode : public rclcpp::Node 
{
public:
    explicit SensorFusingNode();
    ~SensorFusingNode();
private:
    void init();
    void RadarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void fusingInThread();
    void BboxArrayCallback(const ros2_msg::msg::BboxArray::SharedPtr msg);
    cv::Mat GetIntrinsicMatrix();
    cv::Mat GetExtrinsicMatrix();
    void WorldToImage();
    void testFunction();
    void CheckCalibration();
    /* ROS2 sub*/
    void publishInThread(std::vector<fusing_info> objects);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr RadarSubscriber_;
    rclcpp::Subscription<ros2_msg::msg::BboxArray>::SharedPtr BboxArraySubscriber_;
    /* pub */
    rclcpp::Publisher<ros2_msg::msg::FusingArray>::SharedPtr FusingPublisher_;

    std::vector<fusing_info> detected_objects;
    // std::vector<fusing_info> objects;
    std::vector<points> pointcloud;
    std::vector<radar_info> radar_pc;
    std::vector<float> dist;
    
    std::mutex bbox_mutex, radar_mutex;
    std::thread fusingThread_;

    bool RadarStatus_ = false;
    bool BboxStatus_ = false;
    struct timeval startTime_;
    float scale;


    cv::Mat TransformMat_;

    float cam_x, cam_y, cam_z, cam_pitch, cam_yaw, cam_roll, cam_fov_h, cam_fov_v, focal_length;
    int img_height, img_width;
    float radar_x, radar_y, radar_z, radar_pitch, radar_yaw, radar_roll, radar_fov_h, radar_fov_v, radar_range;
    float offset_x, offset_y, offset_z;
};

}



#endif