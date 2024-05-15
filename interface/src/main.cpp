#include "interface.hpp"

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<interface::interface>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

