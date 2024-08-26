#include "rclcpp/rclcpp.hpp"
#include "test_fusion_lidar_node.hpp"

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<test_fusion::test_fusion_node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}