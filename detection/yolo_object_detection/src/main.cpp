#include "rclcpp/rclcpp.hpp"
#include "yolo_object_detection_node.hpp"

int main(int argc, char* argv[]){
  rclcpp::init(argc, argv);
  //ros::init(argc, argv, "yolo_object_detection_node");
  //ros::NodeHandle nodeHandle("~");
  //yolo_object_detection::YoloObjectDetectionNode YOD(nodeHandle);
  std::shared_ptr<rclcpp::Node> node = std::make_shared<yolo_object_detection_ros2::YoloObjectDetectionNode>();
  //ros::spin();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
