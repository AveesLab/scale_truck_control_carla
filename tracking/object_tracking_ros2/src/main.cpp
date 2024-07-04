#include "rclcpp/rclcpp.hpp"
#include "object_tracking_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<object_tracking_ros2::ObjectTrackingNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}