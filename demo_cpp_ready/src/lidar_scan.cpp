#include <rclcpp/rclcpp.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <std_msgs/msg/string.hpp>

//Alias
using namespace std::placeholders;
using namespace ur_rtde;
using String = std_msgs::msg::String;


class LidarScan : public rclcpp::Node {
    public:
        LidarScan(const std::string& ip) : Node("lidar_scan") {
            robot_ip = ip;
            cmd_sub_ = create_subscription<String>("menu_action", 10, 
            [this](const String::SharedPtr cmd) { cmdCallback(cmd);});
        }
    private:
        void cmdCallback(const String::SharedPtr& cmd) {
            if (cmd -> data == "scan env") {
                std::cout << "scannig area" << std::endl;
                scanEnv();
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

};


int main (int argc, char ** argv) {
    rclcpp::init(argc, argv);
    const std::string robot_ip = "172.16.3.131";
    auto node  = std::make_shared<LidarScan>(robot_ip);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}