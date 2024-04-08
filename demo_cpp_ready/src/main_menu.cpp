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
        
        //move commands 
        command_move = menu_handler_.insert("Move");
        move_home = menu_handler_.insert(command_move, "Home", std::bind(&IntMenu::moveHome, this, _1));
        move_3d_Mouse = menu_handler_.insert(command_move, "3D Mouse", std::bind(&IntMenu::move3DMouse, this, _1));
        move_keyboard = menu_handler_.insert(command_move, "Keyboard", std::bind(&IntMenu::moveKeyboard, this, _1));

        
        
        //Tools commands 
        command_tools = menu_handler_.insert("Tools");
        grinder = menu_handler_.insert(command_tools, "Grinders");
        p_grinder = menu_handler_.insert(grinder, "P-Grinder", std::bind(&IntMenu::getTool, this, _1));
        h_grinder = menu_handler_.insert(grinder, "H-Grinder", std::bind(&IntMenu::getTool, this, _1));
        vacuum = menu_handler_.insert(command_tools, "Vacuum", std::bind(&IntMenu::getTool, this, _1));
        gripper = menu_handler_.insert(command_tools, "Gripper", std::bind(&IntMenu::getTool, this, _1));
        marker = menu_handler_.insert(command_tools, "Marker", std::bind(&IntMenu::getTool, this, _1));

        op4 = menu_handler_.insert("Soon...");
        //clear markers on screen 
        clear_objects = menu_handler_.insert("Clear", std::bind(&IntMenu::clearObjects, this, _1));

    }
    //Detection callback
    void detectSurfaces(const MarkerFeedback::ConstSharedPtr &feedback) {
        if (feedback->menu_entry_id == command_detect_surfaces_)
            execute("detect surfaces");
    }

    //moves Callbacks 
    void moveHome(const MarkerFeedback::ConstSharedPtr &feedback){
        if (feedback->menu_entry_id == move_home)
            execute("move home");
    }
    void move3DMouse(const MarkerFeedback::ConstSharedPtr &feedback) {
        if (feedback -> menu_entry_id == move_3d_Mouse){
            execute("3D Mouse");
        }
    }
    void moveKeyboard(const MarkerFeedback::ConstSharedPtr &feedback){
        if(feedback -> menu_entry_id == move_keyboard){
            execute("Keyboard");
        }
    }

    //clear callback
    void clearObjects(const MarkerFeedback::ConstSharedPtr &feedback){
        if (feedback->menu_entry_id == clear_objects)
            
            execute("grinding");
    }

    //Get tools 
    void getTool (const MarkerFeedback::ConstPtr &feedback) {

    }



    Marker makePlane(const float scale)
    {
        Marker menuType;
        menuType.type = Marker::SPHERE;
        // type, scale, color
        menuType.scale.x = scale * 0.25;
        menuType.scale.y = scale * 0.25;
        menuType.scale.z = scale * 0.25;
        menuType.color.r = 1.0f;
        menuType.color.g = 1.0f;
        menuType.color.b = 1.0f;
        menuType.color.a = 1.0;

        return menuType;
    }
    IntControl makePlaneControl(const Marker &p_)
    {
        IntControl menuControl;
        menuControl.always_visible = true;
        menuControl.markers.push_back(p_);
        menuControl.interaction_mode = IntControl::BUTTON;

        return menuControl;
    }
    IntMarker makeMenuPlane(const std::string &name, const Marker &p_)
    {
        IntMarker menu;
        menu.header.frame_id = "base_link";
        menu.pose.position.x = 1.0;
        menu.pose.position.y = 1.0;
        menu.pose.position.z = 1.0;
        menu.name = name;
        menu.description = "Menu";
        menu.scale = 0.25;

        menu.controls.push_back(makePlaneControl(p_));

        // Arrow movement

        IntControl control;
        control.orientation.w = 1;
        // move x
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "move_x";
        control.interaction_mode = IntControl::MOVE_AXIS;
        menu.controls.push_back(control);
        //move y
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "move_y";
        control.interaction_mode = IntControl::MOVE_AXIS;
        menu.controls.push_back(control);
        // move y
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "move_z";
        control.interaction_mode = IntControl::MOVE_AXIS;
        menu.controls.push_back(control);

        // rotation controls
        IntControl rotateX_control;
        rotateX_control.name = "rotate_x";
        rotateX_control.interaction_mode = IntControl::ROTATE_AXIS;
        rotateX_control.orientation.w = 1;
        rotateX_control.orientation.x = 1;
        rotateX_control.orientation.y = 0;
        rotateX_control.orientation.z = 0;
        menu.controls.push_back(rotateX_control);

        IntControl rotateY_control;
        rotateY_control.name = "rotate_x";
        rotateY_control.interaction_mode = IntControl::ROTATE_AXIS;
        rotateY_control.orientation.w = 1;
        rotateY_control.orientation.x = 0;
        rotateY_control.orientation.y = 1;
        rotateY_control.orientation.z = 0;
        menu.controls.push_back(rotateY_control);

        IntControl rotateZ_control;
        rotateZ_control.name = "rotate_x";
        rotateZ_control.interaction_mode = IntControl::ROTATE_AXIS;
        rotateZ_control.orientation.w = 1;
        rotateZ_control.orientation.x = 1;
        rotateZ_control.orientation.y = 0;
        rotateZ_control.orientation.z = 0;
        menu.controls.push_back(rotateZ_control);

        return menu;
    }
    // Declarations
    std::unique_ptr<Server> server_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    Menu menu_handler_;
    Menu::EntryHandle command_detect_surfaces_, 
                      command_move, move_home, move_3d_Mouse, move_keyboard,  
                      command_tools, grinder, p_grinder, h_grinder, vacuum, gripper, marker, 
                      op4,
                      clear_objects;
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