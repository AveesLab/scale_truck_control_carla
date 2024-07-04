#include "sensor_fusing_node.hpp"

namespace sensor_fusing_ros2 {

SensorFusingNode::SensorFusingNode()
    : Node("sensor_fusing_node", rclcpp::NodeOptions()
                .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true))
{
    // if (!readParameters()) {
    //     rclcpp::shutdown();
    // }
    init();
}

SensorFusingNode::~SensorFusingNode()
{

}

void SensorFusingNode::init() {
    RCLCPP_INFO(this->get_logger(), "SensorFusing_Init Start");
    gettimeofday(&startTime_, NULL);

    /* SUB */
    std::string RadarTopicName;
    int RadarQueueSize;
    std::string BboxArrayTopicName;
    int BboxArrayQueueSize;

    /* PUB */
    std::string FusingArrayTopicName;
    int FusingArrayQueueSize;

    /******************************/
    /* Ros Topic Subscribe Option */
    /******************************/
    this->get_parameter_or("subscribers/radar/topic", RadarTopicName, std::string("front_radar")); // radar
    this->get_parameter_or("subscribers/radar/queue_size", RadarQueueSize, 1);
    this->get_parameter_or("subscribers/BboxArray/topic", BboxArrayTopicName, std::string("/yolo_object_detection/BboxArray"));
    this->get_parameter_or("subscribers/BboxArray/queue_size", BboxArrayQueueSize, 10);

    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.best_effort();

    RadarSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(RadarTopicName, qos, std::bind(&SensorFusingNode::RadarCallback, this, std::placeholders::_1));
    
    BboxArraySubscriber_ = this->create_subscription<ros2_msg::msg::BboxArray>(BboxArrayTopicName, qos, std::bind(&SensorFusingNode::BboxArrayCallback, this, std::placeholders::_1));
    
    /******************************/
    /* Ros Topic Publish Option */
    /******************************/
    this->get_parameter_or("publishers/FusingArray/topic", FusingArrayTopicName, std::string("/sensor_fusing/FusingArray"));
    this->get_parameter_or("publishers/FusingArray/queue_size", FusingArrayQueueSize, 1);

    FusingPublisher_ = this->create_publisher<ros2_msg::msg::FusingArray>(FusingArrayTopicName, qos);

    // cv::Mat IntrinsicMat(3, 3, cv::CV_32F);

    
    /*************************/
    /* Get Camera Parameters */
    /*************************/ 
    // this->get_parameter_or("rgbcam/x", cam_x, 2.0);
    // this->get_parameter_or("rgbcam/y", cam_y, 0.0);
    // this->get_parameter_or("rgbcam/z", cam_z, 3.5);
    // this->get_parameter_or("rgbcam/pitch", cam_pitch, -15.0); // y
    // this->get_parameter_or("rgbcam/yaw", cam_yaw, 0.0); // x
    // this->get_parameter_or("rgbcam/roll", cam_roll, 0.0); // z
    // this->get_parameter_or("rgbcam/image_size_x", img_width, 640);
    // this->get_parameter_or("rgbcam/image_size_y", img_height, 480);
    
    // this->get_parameter_or("rgbcam/fov", cam_fov_h, 90.0);

    /************************/
    /* Get Radar Parameters */
    /************************/
    // this->get_parameter_or("radar/x", radar_x, 2.3);
    // this->get_parameter_or("radar/y", radar_y, 0.0);
    // this->get_parameter_or("radar/z", radar_z, 1.5);
    // this->get_parameter_or("radar/pitch", radar_pitch, 0.0);
    // this->get_parameter_or("radar/yaw", radar_yaw, 0.0);
    // this->get_parameter_or("radar/roll", radar_roll, 0.0);
    // this->get_parameter_or("radar/horizontal_fov", radar_fov_h, 40.0);
    // this->get_parameter_or("radar/vertical_fov", radar_fov_v, 30.0);
    // this->get_parameter_or("radar/range", radar_range, 300.0);

    cam_yaw = 0.0;
    cam_pitch = -15.0;
    cam_roll = 0.0;
    img_width = 640;
    img_height = 480;
    cam_fov_h = 90.0;
    /***************/
    /* Start Setup */
    /***************/
    focal_length = img_width / (2 * std::tan(cam_fov_h / 2));
    cam_fov_v = std::atan(img_height / (2 * focal_length));

    // SersorCalibration();
    cv::Mat intrinsicMat = GetIntrinsicMatrix();

    cv::Mat extrinsicMat = GetExtrinsicMatrix();
    
    TransformMat_ = intrinsicMat * extrinsicMat;
    // cv::Mat inverseMat = TransformMat_.inv();
    fusingThread_ = std::thread(&SensorFusingNode::fusingInThread, this);
}

void SensorFusingNode::BboxArrayCallback(const ros2_msg::msg::BboxArray::SharedPtr msg)
{   
    std::scoped_lock lock(bbox_mutex);
    detected_objects.clear();
    fusing_info b;
    for (auto &box : msg->boxes) {
        b.class_id = box.class_id;
        b.x = (float)box.x;
        b.y = (float)box.y;
        b.w = (float)box.w;
        b.h = (float)box.h;
        detected_objects.push_back(b);
    }
    BboxStatus_ = true;
    // RCLCPP_INFO(this->get_logger(),"size : %d", detected_objects.size());
}


void SensorFusingNode::RadarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{   
    std::scoped_lock lock(radar_mutex);
    radar_pc.clear();
    radar_info r;

    auto point_step = msg->point_step;
    auto data = msg->data;

    for (size_t i = 0; i < msg->width; ++i) {
        float x, y, z, range, velocity, azimuth_angle, elevation_angle;

        memcpy(&x, &data[i * point_step + msg->fields[0].offset], sizeof(float));
        memcpy(&y, &data[i * point_step + msg->fields[1].offset], sizeof(float));
        memcpy(&z, &data[i * point_step + msg->fields[2].offset], sizeof(float));
        memcpy(&range, &data[i * point_step + msg->fields[3].offset], sizeof(float));
        memcpy(&velocity, &data[i * point_step + msg->fields[4].offset], sizeof(float));
        memcpy(&azimuth_angle, &data[i * point_step + msg->fields[5].offset], sizeof(float));
        memcpy(&elevation_angle, &data[i * point_step + msg->fields[6].offset], sizeof(float));
        r.x = z;
        r.y = y - 2.0;
        r.z = x;
        r.range = range;
        r.velocity = velocity;
        r.azimuth = azimuth_angle;
        radar_pc.push_back(r);
        // RCLCPP_INFO(this->get_logger(),
        //             "Radar data - x: %f, y: %f, z: %f, range: %f, velocity: %f, azimuth_angle: %f, elevation_angle: %f",
        //             r.x, r.y, r.z, range, velocity, azimuth_angle, elevation_angle);
    }
    
    // int row_step = msg->row_step;
    // int h = msg->height;
    // int w = msg->width;
    // int cnt = 0;
    // auto d = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Check");
    // for (int idx = 0; idx < row_step; idx+=7) {
    //     r.x = d[idx+2];
    //     r.y = d[idx+1];
    //     r.z = d[idx];
    //     r.range = d[idx+3];
    //     r.velocity = d[idx+4];
    //     r.azimuth = d[idx+5];
    //     radar_pc.push_back(r);
        // if (r.x < 4.0f && r.range > 1.0f) {
        // RCLCPP_INFO(this->get_logger(),"point_cloud : [%f, %f, %f, %f, %f, %f]", r.x, r.y, r.z, r.range, r.velocity, r.azimuth);
        // }
    // }
    RadarStatus_ = true;
    
    // std::cout << "cnt : " << cnt << std::endl;
    // std::cout << "point_step: " << msg->point_step << std::endl;
    // std::cout << "row_step: " << msg->row_step << std::endl;
    // std::cout << "height: " << msg->height << std::endl;
    // std::cout << "width: " << msg->width << std::endl;
    // RCLCPP_INFO(this->get_logger(),"point_cloud : [%f, %f, %f, %f, %f]", r.x, r.y, r.z, r.range, r.velocity);
}
cv::Mat SensorFusingNode::GetIntrinsicMatrix()
{
    cv::Mat intrinsicMat(3, 4, CV_32F);

    intrinsicMat.at<float>(0, 0) = focal_length;
    intrinsicMat.at<float>(0, 1) = 0; // skew_a
    intrinsicMat.at<float>(0, 2) = img_width/2;
    intrinsicMat.at<float>(0, 3) = 0;
    intrinsicMat.at<float>(1, 0) = 0;
    intrinsicMat.at<float>(1, 1) = focal_length;
    intrinsicMat.at<float>(1, 2) = img_height / 2;
    intrinsicMat.at<float>(1, 3) = 0;
    intrinsicMat.at<float>(2, 0) = 0;
    intrinsicMat.at<float>(2, 1) = 0;
    intrinsicMat.at<float>(2, 2) = 1;
    intrinsicMat.at<float>(2, 3) = 0;

    return intrinsicMat;
}

cv::Mat SensorFusingNode::GetExtrinsicMatrix()
{
    cv::Mat extrinsicMat(4, 4, CV_32F);
    /* Rotation */
    extrinsicMat.at<float>(0, 0) = std::cos(cam_yaw) * std::cos(cam_pitch);
    extrinsicMat.at<float>(0, 1) = std::cos(cam_yaw) * std::sin(cam_pitch) * std::sin(cam_roll) + std::sin(cam_yaw) * std::cos(cam_roll);
    extrinsicMat.at<float>(0, 2) = -std::cos(cam_yaw) * std::sin(cam_pitch) * std::cos(cam_roll) + std::sin(cam_yaw) * std::sin(cam_roll);
    
    extrinsicMat.at<float>(1, 0) = -std::sin(cam_yaw) * std::sin(cam_pitch);
    extrinsicMat.at<float>(1, 1) = -std::sin(cam_yaw) * std::sin(cam_pitch) * std::sin(cam_roll) + std::cos(cam_yaw) * std::cos(cam_roll);
    extrinsicMat.at<float>(1, 2) = std::sin(cam_yaw) * std::sin(cam_pitch) * std::cos(cam_roll) + std::cos(cam_yaw) * std::sin(cam_roll);
    
    extrinsicMat.at<float>(2, 0) = std::sin(cam_pitch);
    extrinsicMat.at<float>(2, 1) = -std::cos(cam_pitch) * std::sin(cam_roll);
    extrinsicMat.at<float>(2, 2) = std::cos(cam_pitch) * std::cos(cam_roll);
    
    /* Translation */
    extrinsicMat.at<float>(0, 3) = 0;
    extrinsicMat.at<float>(1, 3) = 0;
    extrinsicMat.at<float>(2, 3) = 0;

    extrinsicMat.at<float>(3, 0) = 0;
    extrinsicMat.at<float>(3, 1) = 0;
    extrinsicMat.at<float>(3, 2) = 0;
    extrinsicMat.at<float>(3, 3) = 1;
    
    return extrinsicMat;
}

void SensorFusingNode::CheckCalibration()
{
    for (auto &p : radar_pc) {
        cv::Mat world_points(4, 1, CV_32F);
        world_points.at<float>(0, 0) = p.x;
        world_points.at<float>(1, 0) = p.y;
        world_points.at<float>(2, 0) = p.z;
        world_points.at<float>(3, 0) = 1.0f;
        // RCLCPP_INFO(this->get_logger(), "CHECKING WORLD POINT");
        // RCLCPP_INFO(this->get_logger(), "X : %f -> %f, Y : %f -> %f, Z : %f -> %f", p.x, world_points.at<float>(0, 0), p.y, world_points.at<float>(1, 0), p.z, world_points.at<float>(2, 0));
    }
}



void SensorFusingNode::WorldToImage()
{   
    if (!detected_objects.empty()) {
        // RCLCPP_INFO(this->get_logger(), "Start."); 
        for (auto &b : detected_objects) {
            b.distance = 99999999.0f;
            float min_x = b.x - b.w / 2;
            float max_x = b.x + b.w / 2;
            float min_y = b.y - b.h / 2;
            float max_y = b.y + b.h / 2;
            float tmp_x, tmp_y, x2, y2;
            for (auto &p : radar_pc) {
                cv::Mat world_points(4, 1, CV_32F);
                world_points.at<float>(0, 0) = p.x;
                world_points.at<float>(1, 0) = p.y;
                world_points.at<float>(2, 0) = p.z;
                world_points.at<float>(3, 0) = 1.0f;
                cv::Mat image_points = TransformMat_ * world_points; //->> process died;
                scale = image_points.at<float>(2, 0);
                float y = image_points.at<float>(0, 0) / scale;

                float x = image_points.at<float>(1, 0) / scale;
                // if (p.range < 15.0 && p.range > 13.0) {
                // RCLCPP_INFO(this->get_logger(), "x : %f, y : %f, scale : %f, range : %f, speed : %f", x, y, scale, p.range, p.velocity);
                // RCLCPP_INFO(this->get_logger(), "x_min : %f, x_max : %f, y_min : %f, y_max : %f", min_x, max_x, min_y, max_y);
                if (x < min_x || x > max_x || y < min_y || y > max_y) {
                    continue;
                    }
                else {
                    b.distance = std::min(b.distance, p.range);
                    if (p.velocity > -10.0f) {
                        b.velocity = p.velocity;
                        x2 = x;
                        y2 = y;
                    }
                    tmp_x = x;
                    tmp_y = y;  
                }
            }
            // RCLCPP_INFO(this->get_logger(), "ID : %d, Distance : %f, Speed : %f", b.class_id, b.distance, b.velocity); 
            // RCLCPP_INFO(this->get_logger(), "x : %f, y: %f, x: %f, y: %f", tmp_x, tmp_y, x2, y2); 
        }
    }
}


void SensorFusingNode::testFunction() {
    dist.clear();
    std::cout << "ok" << std::endl; 
}

// Edit //
void SensorFusingNode::publishInThread(std::vector<fusing_info> objects)
{
    ros2_msg::msg::Fusing fusingbox;
    ros2_msg::msg::FusingArray fusingboxes;
    // RCLCPP_INFO(this->get_logger(), "Pub fusing_info");
    for (auto &obj : objects) {
        fusingbox.x = obj.x;
        fusingbox.y = obj.y;
        fusingbox.w = obj.w;
        fusingbox.h = obj.h;
        fusingbox.class_id = obj.class_id;
        fusingbox.distance = obj.distance;
        fusingbox.velocity = obj.velocity;
        fusingboxes.fusingarr.push_back(fusingbox);
    }
    FusingPublisher_->publish(fusingboxes);
    // RCLCPP_INFO(this->get_logger(), "CheckPoint3");
}

void SensorFusingNode::fusingInThread()
{
    RCLCPP_INFO(this->get_logger(), "fusingInThread Start");
    struct timeval endTime;
    static double time = 0.0;

    while (rclcpp::ok()) {
        if (BboxStatus_ && RadarStatus_) {
            std::scoped_lock lock(bbox_mutex, radar_mutex);
            WorldToImage();
            // CheckCalibration();
            RadarStatus_ = false;
            BboxStatus_ = false;
            publishInThread(detected_objects);
        }
        // RCLCPP_INFO(this->get_logger(), "%d, %d", detected_objects.size(), radar_pc.size());
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
    }
}

}