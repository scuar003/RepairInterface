#ifndef ROS2_MENU_H
#define ROS2_MENU_H

#include <memory>
#include <string>
#include <vector>
#include <sstream>

//Ros2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

//Alias 
using namespace std::placeholders;
using Marker = visualization_msgs::msg::Marker; // Static object
using IntMarker = visualization_msgs::msg::InteractiveMarker; // Interactive object 
using IntControl = visualization_msgs::msg::InteractiveMarkerControl; // Interactive control for interactive object
using MarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback; // Use for button or action on markers (gets the feedback)
using IntServer = interactive_markers::InteractiveMarkerServer;
using Menu = interactive_markers::MenuHandler;
using StringRos2 = std_msgs::msg::String;

class MainMenu : public rclcpp::Node {
    public: 
        MainMenu();
        void init();
    private:
        void execute(std::string cmd);
        void startMenu ();
        void menuAction (const MarkerFeedback::ConstSharedPtr &feedback);
        Marker makeMarker (const float scale);
        IntMarker makeMarkerMenu(const std::string &name, const Marker &p);
        IntControl  makeMarkerControl (const Marker &p);



        std::unique_ptr<IntServer> interactive_server_;
        rclcpp::Publisher<StringRos2>::SharedPtr command_pub;
        Menu menu_handler;
        Menu::EntryHandle menu_repair, detect_surfaces, scan_env, 
                          menu_move, move_home, move_3d_Mouse, move_keyboard,  
                          menu_tools, grinder, p_grinder, h_grinder, vacuum, gripper, marker,
                          next_entry;
            
};

#endif