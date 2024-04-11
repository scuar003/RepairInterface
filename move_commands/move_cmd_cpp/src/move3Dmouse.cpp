#include <iostream>
#include <string>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

//alias
using namespace std::placeholders;


class Move3DMouse : public rclcpp::Node {
    public:
        //constructor
        Move3DMouse() : Node("move_Mouse_node") {
            subscriber_menu_action = create_subscription<std_msgs::msg::String>("menu_action", 10,std::bind(&Move3DMouse::menuActionCallback,this, _1));

        }

    private:
        void menuActionCallback(const std_msgs::msg::String::SharedPtr msg) {
            if(msg->data == "3D Mouse") {
                RCLCPP_INFO(this->get_logger(), "Activating 3D mouse");

                system("/home/robotics/development/Demo/src/move_commands/move_cmd_cpp/src/moveitservo3dmouse.bash");
            }
        }
        
        //Declarations
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_menu_action;
};

int main (int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Move3DMouse>());
    rclcpp::shutdown();

    return 0;
}
