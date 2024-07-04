#include "rclcpp/rclcpp.hpp"
#include "sensor_fusing_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<sensor_fusing_ros2::SensorFusingNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}