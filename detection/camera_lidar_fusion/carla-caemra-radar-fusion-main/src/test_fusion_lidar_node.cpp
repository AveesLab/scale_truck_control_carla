#include "test_fusion_lidar_node.hpp"
#include <cmath>

namespace test_fusion
{

test_fusion_node::test_fusion_node()
    : Node("test_fusion_node", rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true))
{
    // QoS 설정
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));


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


    ImageSubscriber_ = this->create_subscription<sensor_msgs::msg::Image>("camera0",qos,bind(&test_fusion_node::ImageSubCallback,this,std::placeholders::_1));
    BboxArraySubscriber_ = this->create_subscription<ros2_msg::msg::BboxArray>("yolo_object_detection/BboxArray",qos,std::bind(&test_fusion_node::BboxArraySubCallback, this, std::placeholders::_1));
    PointcloudSubscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("detected_vehicles",qos,std::bind(&test_fusion_node::PointcloudSubCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(50ms, std::bind(&test_fusion_node::timerCallback, this));
    //FusingPublisher_ = this->create_publisher<ros2_msg::msg::FusingArray>("sensor_fusing/FusingArray", 1);
}


test_fusion_node::~test_fusion_node()
{

}

void test_fusion_node::ImageSubCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cam_image;
    try {
        cam_image = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception : %s",e.what());
    }

    this->image_ = cam_image->image;
    isImageReady = true;
}

void test_fusion_node::BboxArraySubCallback(const ros2_msg::msg::BboxArray::SharedPtr msg) {
    bbox_msg = *msg;
    isBboxReady = true;
}
void test_fusion_node::PointcloudSubCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    points_msg = *msg;
    isPointsReady = true;
}

bool test_fusion_node::isDataReady() {
    if(!isBboxReady) return false;
    if(!isPointsReady) return false;
    if(!isImageReady) return false;

    isImageReady = false;
    isBboxReady = false;
    isPointsReady = false;
    return true;
}

void test_fusion_node::timerCallback() {
    if(!isDataReady()) return;
    std::cerr << "fusion publish" << std::endl;

    std::map<int, std::tuple<double, double, cv::Point>> closest_points; // name -> (min_distance, velocity, point)

    // 포인트 클라우드 데이터를 순회하며 변환 및 오버레이
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(points_msg, "x"),
                                                      iter_y(points_msg, "y"),
                                                      iter_z(points_msg, "z");
         iter_x != iter_x.end();
         ++iter_x, ++iter_y, ++iter_z)
    {
        // 레이더 좌표계를 카메라 좌표계로 변환 (레이더의 x -> 카메라의 z, 레이더의 y -> 카메라의 x, 레이더의 z -> 카메라의 y)
        cv::Mat radar_point = (cv::Mat_<double>(4, 1) << *iter_x, *iter_y, *iter_z, 1.0);
        cv::Mat cam_point = transform_ * radar_point;

        double cam_z = cam_point.at<double>(0, 0); // 레이더의 x -> 카메라의 z
        double cam_x = cam_point.at<double>(1, 0); // 레이더의 y -> 카메라의 x
        double cam_y = cam_point.at<double>(2, 0); // 레이더의 z -> 카메라의 y

        // 카메라 좌표를 이미지 좌표로 변환
        cv::Mat uvw = camera_matrix_ * (cv::Mat_<double>(3, 1) << -cam_x, -cam_y, cam_z); // cam_y 값을 음수로 변환하여 이미지 y 좌표의 방향을 맞춤
        int img_x = static_cast<int>(uvw.at<double>(0, 0) / uvw.at<double>(2, 0));
        int img_y = static_cast<int>(uvw.at<double>(1, 0) / uvw.at<double>(2, 0));

        // 이미지 범위 내에 있는지 확인
        if (img_x >= 0 && img_x < 640 && img_y >= 0 && img_y < 480)
        {
            // 각 바운딩 박스 내에서 가장 가까운 포인트를 찾기
            for (const auto& bbox : bbox_msg.boxes)
            {
                if (img_x >= bbox.x && img_x < bbox.x + bbox.w &&
                    img_y >= bbox.y && img_y < bbox.y + bbox.h)
                {
                    double distance = 0.0; // 레이더 포인트의 깊이 값을 거리로 사용
                    double vel = 0.0;
                    if (closest_points.find(bbox.class_id) == closest_points.end() || distance < std::get<0>(closest_points[bbox.class_id]))
                    {
                        closest_points[bbox.class_id] = std::make_tuple(distance, vel, cv::Point(img_x, img_y));
                    }
                }
            }

            cv::circle(image_, cv::Point(img_x,img_y),2,cv::Scalar(0,0,255), -1);
        }
    }

  for(auto &bbox : bbox_msg.boxes)
  {
    cv::rectangle(image_, cv::Rect(bbox.x - bbox.w / 2 , bbox.y - bbox.h / 2 , bbox.w, bbox.h), (255,0,0), 2);
  }

    cv::namedWindow("Fused Image");
    cv::imshow("Fused Image",image_);
    cv::waitKey(10);

    /*

    // 각 바운딩 박스에 대해 가장 가까운 포인트 출력
    ros2_msg::msg::Fusing fusingbox;
    ros2_msg::msg::FusingArray fusingboxes;

    for (const auto& bbox : bbox_msg.boxes)
    {
        if (closest_points.find(bbox.class_id) != closest_points.end())
        {
            const auto& [distance, velocity, point] = closest_points[bbox.class_id];
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

    std::cerr << "fusion publish end" << std::endl;
    FusingPublisher_->publish(fusingboxes);
    */
}


void test_fusion_node::callback(const ros2_msg::msg::BboxArray::ConstSharedPtr bbox_msg, const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg)
{

}


}