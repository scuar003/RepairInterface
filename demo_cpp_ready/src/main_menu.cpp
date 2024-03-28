#include <memory>
#include <string>
#include <vector>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

// alias
using namespace std::placeholders;
using Marker = visualization_msgs::msg::Marker;
using Server = interactive_markers::InteractiveMarkerServer;
using IntControl = visualization_msgs::msg::InteractiveMarkerControl;
using IntMarker = visualization_msgs::msg::InteractiveMarker;
using Menu = interactive_markers::MenuHandler;
using MarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;

class IntMenu : public rclcpp::Node {
public:
    IntMenu() : Node("IntMenu") {}

    void init() {
        server_ = std::make_unique<Server>("menu_server", shared_from_this());
        startMenu();
        auto marker = makePlane(1.0);
        server_->insert(makeMenuPlane("IntMenu", marker));
        menu_handler_.apply(*server_, "IntMenu");
        server_->applyChanges();
        // Commands
        command_publisher_ = create_publisher<std_msgs::msg::String>("repair_command", 10);
    }

private:

    void execute(std::string cmd) {
        std_msgs::msg::String command;
        command.data = cmd;
        command_publisher_->publish(command);
    }

    void startMenu() {
        command_detect_surfaces_ = menu_handler_.insert("Detect Surfaces", std::bind(&IntMenu::detectSurfaces, this, _1));
        op1 = menu_handler_.insert("Cut Pipe", std::bind(&IntMenu::cutPipe, this, _1));
        op2 = menu_handler_.insert("Grind Angle Surface", std::bind(&IntMenu::angleSurface, this, _1));
        op3 = menu_handler_.insert("Soon...");
        op4 = menu_handler_.insert("Soon...");

        // Menu::EntryHandle sub_menu = menu_handler_.insert("switch");
        // std::vector<std::string>types{"PLANE", "SPHERE", "CYLINDER", "CUBE"};

        // for (int i = 0; i <4; ++i)
        // {
        //     std::ostringstream s;
        //     s << types[i];
        //     next_entry = menu_handler_.insert(sub_menu, s.str(), std::bind(&IntMenu::modeCallback, this, _1));
        //     menu_handler_.setCheckState(next_entry, Menu::UNCHECKED);
        // }

        // menu_handler_.setCheckState(next_entry, Menu::CHECKED);
    }
    void detectSurfaces(const MarkerFeedback::ConstSharedPtr &feedback) {
        if (feedback->menu_entry_id == command_detect_surfaces_)
            execute("detect surfaces");
    }
    void cutPipe(const MarkerFeedback::ConstSharedPtr &feedback){
        if (feedback->menu_entry_id == op1)
            execute("cut pipe");
    }
    void angleSurface(const MarkerFeedback::ConstSharedPtr &feedback){
        if (feedback->menu_entry_id == op2)
            execute("grinding");
    }

    Marker makePlane(const float scale)
    {
        Marker pType;
        pType.type = Marker::SPHERE;
        // type, scale, color
        pType.scale.x = scale * 0.45;
        pType.scale.y = scale * 0.45;
        pType.scale.z = scale * 0.45;
        pType.color.r = 1.0f;
        pType.color.g = 1.0f;
        pType.color.b = 1.0f;
        pType.color.a = 1.0;

        return pType;
    }
    IntControl makePlaneControl(const Marker &p_)
    {
        IntControl planeControl;
        planeControl.always_visible = true;
        planeControl.markers.push_back(p_);
        planeControl.interaction_mode = IntControl::BUTTON;

        return planeControl;
    }
    IntMarker makeMenuPlane(const std::string &name, const Marker &p_)
    {
        IntMarker plane;
        plane.header.frame_id = "base_link";
        plane.pose.position.x = 1.0;
        plane.pose.position.y = 1.0;
        plane.pose.position.z = 1.0;
        plane.name = name;
        plane.description = "Menu";
        plane.scale = 0.50;

        plane.controls.push_back(makePlaneControl(p_));

        // Arrow movement

        IntControl control;
        control.orientation.w = 1;
        // move x
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "move_x";
        control.interaction_mode = IntControl::MOVE_AXIS;
        plane.controls.push_back(control);
        //move y
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "move_y";
        control.interaction_mode = IntControl::MOVE_AXIS;
        plane.controls.push_back(control);
        // move y
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "move_z";
        control.interaction_mode = IntControl::MOVE_AXIS;
        plane.controls.push_back(control);

        // rotation controls
        IntControl rotateX_control;
        rotateX_control.name = "rotate_x";
        rotateX_control.interaction_mode = IntControl::ROTATE_AXIS;
        rotateX_control.orientation.w = 1;
        rotateX_control.orientation.x = 1;
        rotateX_control.orientation.y = 0;
        rotateX_control.orientation.z = 0;
        plane.controls.push_back(rotateX_control);

        IntControl rotateY_control;
        rotateY_control.name = "rotate_x";
        rotateY_control.interaction_mode = IntControl::ROTATE_AXIS;
        rotateY_control.orientation.w = 1;
        rotateY_control.orientation.x = 0;
        rotateY_control.orientation.y = 1;
        rotateY_control.orientation.z = 0;
        plane.controls.push_back(rotateY_control);

        IntControl rotateZ_control;
        rotateZ_control.name = "rotate_x";
        rotateZ_control.interaction_mode = IntControl::ROTATE_AXIS;
        rotateZ_control.orientation.w = 1;
        rotateZ_control.orientation.x = 1;
        rotateZ_control.orientation.y = 0;
        rotateZ_control.orientation.z = 0;
        plane.controls.push_back(rotateZ_control);

        return plane;
    }
    // Declarations
    std::unique_ptr<Server> server_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    Menu menu_handler_;
    Menu::EntryHandle command_detect_surfaces_, op1, op2, op3, op4;
    Menu::EntryHandle next_entry;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IntMenu>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}