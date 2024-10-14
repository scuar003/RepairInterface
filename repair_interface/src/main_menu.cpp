#include "repair_interface/ros2_menu.h"

int main (int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainMenu>();
    node -> init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;  
}