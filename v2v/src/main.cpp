#include "v2v.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<v2v::V2V>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

