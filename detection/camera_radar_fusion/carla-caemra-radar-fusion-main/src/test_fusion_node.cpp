#include "test_fusion_node.hpp"
#include <cmath>

namespace test_fusion
{

test_fusion_node::test_fusion_node()
    : Node("test_fusion_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true)),
           bbox_sub_(this, "yolo_object_detection/BboxArray", rclcpp::SensorDataQoS().get_rmw_qos_profile()),
      pointcloud_sub_(this, "clustered_radar_points", rclcpp::SensorDataQoS().get_rmw_qos_profile())
{
    // QoS 설정
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos.best_effort();

            // Subscriber 초기화

    // Synchronizer 초기화 (ExactTime 정책 사용)
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), bbox_sub_, pointcloud_sub_);
    sync_->registerCallback(std::bind(&test_fusion_node::callback, this, std::placeholders::_1, std::placeholders::_2));
    // 카메라 내부 파라미터 설정
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 320, 0, 320, 0, 320, 240, 0, 0, 1); // focal_length = 320, cx = 320, cy = 240
    dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F); // 렌즈 왜곡 없음

    // 카메라와 레이더 간 변환 매트릭스 설정
    double theta = -15.0 * M_PI / 180.0; // -15도 각도 라디안으로 변환
    cv::Mat rotation = (cv::Mat_<double>(3, 3) <<
                         cos(theta), 0, sin(theta),
                         0, 1, 0,
                         -sin(theta), 0, cos(theta));
    cv::Mat translation = (cv::Mat_<double>(3, 1) << 0, 0, -2.0); // z축 방향으로 2m 이동 (카메라가 레이더보다 위에 있음)

    // 변환 매트릭스를 4x4 동차 좌표계로 확장
    transform_ = cv::Mat::eye(4, 4, CV_64F);
    rotation.copyTo(transform_(cv::Rect(0, 0, 3, 3)));
    translation.copyTo(transform_(cv::Rect(3, 0, 1, 3)));
    FusingPublisher_ = this->create_publisher<ros2_msg::msg::FusingArray>("sensor_fusing/FusingArray", 1);
}


test_fusion_node::~test_fusion_node()
{

}


void test_fusion_node::callback(const ros2_msg::msg::BboxArray::ConstSharedPtr bbox_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg)
{
   
    std::map<int, std::tuple<double, double, cv::Point>> closest_points; // name -> (min_distance, velocity, point)    // 포인트 클라우드 데이터를 순회하며 변환 및 오버레이
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*points_msg, "x"),
                                                      iter_y(*points_msg, "y"),
                                                      iter_z(*points_msg, "z"),
                                                      iter_range(*points_msg, "Range"),
                                                      iter_velocity(*points_msg, "Velocity");
         iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z, ++iter_range, ++iter_velocity)
    {
        // 레이더 좌표계를 카메라 좌표계로 변환 (레이더의 x -> 카메라의 z, 레이더의 y -> 카메라의 x, 레이더의 z -> 카메라의 y)
        cv::Mat radar_point = (cv::Mat_<double>(4, 1) << *iter_x, *iter_y, *iter_z, 1.0);
        cv::Mat cam_point = transform_ * radar_point;        
        double cam_z = cam_point.at<double>(0, 0); // 레이더의 x -> 카메라의 z
        double cam_x = cam_point.at<double>(1, 0); // 레이더의 y -> 카메라의 x
        double cam_y = cam_point.at<double>(2, 0); // 레이더의 z -> 카메라의 y        // 카메라 좌표를 이미지 좌표로 변환
        cv::Mat uvw = camera_matrix_ * (cv::Mat_<double>(3, 1) << -cam_x, -cam_y, cam_z); // cam_y 값을 음수로 변환하여 이미지 y 좌표의 방향을 맞춤
        int img_x = static_cast<int>(uvw.at<double>(0, 0) / uvw.at<double>(2, 0));
        int img_y = static_cast<int>(uvw.at<double>(1, 0) / uvw.at<double>(2, 0));        // 이미지 범위 내에 있는지 확인
        
        if (img_x >= 0 && img_x < 640 && img_y >= 0 && img_y < 480)
        {
            // 각 바운딩 박스 내에서 가장 가까운 포인트를 찾기
            for (const auto& bbox : bbox_msg->boxes)
            {
                if (img_x >= bbox.x && img_x < bbox.x + bbox.w &&
                    img_y >= bbox.y && img_y < bbox.y + bbox.h)
                {
                    double distance = *iter_range; // 레이더 포인트의 깊이 값을 거리로 사용
                    double vel = *iter_velocity;
                    if (closest_points.find(bbox.class_id) == closest_points.end() || distance < std::get<0>(closest_points[bbox.class_id]))
                    {
                        closest_points[bbox.class_id] = std::make_tuple(distance, vel, cv::Point(img_x, img_y));
                    }
                }
            }
        }
    }    // 각 바운딩 박스에 대해 가장 가까운 포인트 출력

    ros2_msg::msg::Fusing fusingbox;
    ros2_msg::msg::FusingArray fusingboxes;   

    for (const auto& bbox : bbox_msg->boxes)
    {
        if (closest_points.find(bbox.class_id) != closest_points.end())
        {
            const auto& [distance, velocity, point] = closest_points[bbox.class_id];
            //RCLCPP_INFO(this->get_logger(), "Bounding Box: %d, Closest Point: (%d, %d), Distance: %f, Velocity: %lf", bbox.class_id, point.x, point.y, distance, velocity);
            fusingbox.x = bbox.x;
            fusingbox.y = bbox.y;
            fusingbox.w = bbox.w;
            fusingbox.h = bbox.h;
            fusingbox.class_id = bbox.class_id;
            fusingbox.distance = distance;
            fusingbox.velocity = velocity;
            fusingboxes.fusingarr.push_back(fusingbox);
        }
    } 
    FusingPublisher_->publish(fusingboxes);
}


}