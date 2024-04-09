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

                system("source ~/Projects/ws_ur/install/setup.bash && ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur16e launch_rviz:=false &");
                system("source ~/Projects/ws_ur/install/setup.bash && ros2 run spacenav spacenav_node &");
                system("source ~/Projects/ws_ur/install/setup.bash && ros2 run ur_robot_driver twist_relay_node &");
                system("ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \"{activate_controllers: ['forward_position_controller'], deactivate_controllers: ['scaled_joint_trajectory_controller'], strictness: 1}\" &");
                system("ros2 service call /servo_node/start_servo std_srvs/srv/Trigger &");
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
