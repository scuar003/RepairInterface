#include <rclcpp/rclcpp.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <thread>
#include <functional>
#include <string>


//Alias
using namespace std::placeholders;
using namespace ur_rtde;
using namespace std::chrono_literals;

using String = std_msgs::msg::String;


class LidarScan : public rclcpp::Node {
    public:
        LidarScan() : Node("lidar_scan") {
            // Declare parameters with Default parameters 
            this -> declare_parameter ("robot_ip", "192.168.56.101");
            this -> declare_parameter ("menu_topic", "menu_actions");
            this -> declare_parameter ("laser_topic", "/scan");

            // Retrieve Parameters
            this -> get_parameter("robot_ip", robot_ip);
            this -> get_parameter("menu_topic", menu_topic);
            this -> get_parameter("laser_topic", laser_topic);
            
            cmd_sub_ = create_subscription<String>("menu_action", 10, 
            [this](const String::SharedPtr cmd) { cmdCallback(cmd);});
        }
    private:
        bool isLidarActive(const rclcpp::Node::SharedPtr& node, const std::string & laser_topic) {
            auto publishers_info = node->get_node_graph_interface()->get_publishers_info_by_topic(laser_topic);
            return !publishers_info.empty();
        }
        void cmdCallback(const String::SharedPtr& cmd) {
            if (cmd -> data == "scan env") {
                std::cout << "scannig area" << std::endl;
                if(!isLidarActive(shared_from_this(), laser_topic)) {
                    RCLCPP_WARN(get_logger(), "Laser node is not publishing on %s. Aborting scan.", laser_topic.c_str());
                    return;
                } else {
                    scanEnv();
                }
            }
        }
        void scanEnv() {
            RTDEControlInterface robot_(robot_ip);
            auto acc = 0.08, vel = 0.08;
            //Start
            robot_.moveJ({-1.57, -0.94, -1.33, -1.65, 1.55, 0.0}, acc, vel);
            //right
            robot_.moveJ({-3.25, -0.94, -1.33, -1.65, 1.55, 0.0}, acc, vel);
            //left
            robot_.moveJ({0.83, -0.94, -1.33, -1.65, 1.55, 0.0}, acc, vel);
            //end
            robot_.moveJ({-1.57, -0.94, -1.33, -1.65, 1.55, 0.0}, acc, vel);
            std::cout << "Scan Complete" << std::endl;
        }

        rclcpp::Subscription<String>::SharedPtr cmd_sub_;
        std::string robot_ip;
        std::string menu_topic;
        std::string laser_topic; 
        
};

int main (int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node  = std::make_shared<LidarScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}