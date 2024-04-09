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
        command_publisher_ = create_publisher<std_msgs::msg::String>("menu_action", 10);
    }

private:

    void execute(std::string cmd) {
        std_msgs::msg::String command;
        command.data = cmd;
        command_publisher_->publish(command);
    }

    void startMenu() {

        //Actions - Interactions
        menu_repair = menu_handler_.insert("Repair Operations");
        detect_surfaces = menu_handler_.insert(menu_repair, "Detect Surfaces", std::bind(&IntMenu::repairOperation, this, _1));
        

        //move commands 
        menu_move = menu_handler_.insert("Move");
        move_home = menu_handler_.insert(menu_move, "Home", std::bind(&IntMenu::moveCommand, this, _1));
        move_3d_Mouse = menu_handler_.insert(menu_move, "3D Mouse", std::bind(&IntMenu::moveCommand, this, _1));
        move_keyboard = menu_handler_.insert(menu_move, "Keyboard", std::bind(&IntMenu::moveCommand, this, _1));

        
        
        //Tools  
        menu_tools = menu_handler_.insert("Get/Change Tools");
        grinder = menu_handler_.insert(menu_tools, "Grinders");
        p_grinder = menu_handler_.insert(grinder, "P-Grinder", std::bind(&IntMenu::getTool, this, _1));
        h_grinder = menu_handler_.insert(grinder, "H-Grinder", std::bind(&IntMenu::getTool, this, _1));
        vacuum = menu_handler_.insert(menu_tools, "Vacuum", std::bind(&IntMenu::getTool, this, _1));
        gripper = menu_handler_.insert(menu_tools, "Gripper", std::bind(&IntMenu::getTool, this, _1));
        marker = menu_handler_.insert(menu_tools, "Marker", std::bind(&IntMenu::getTool, this, _1));

        //Dashboard options 
        op4 = menu_handler_.insert("Soon...");

        //clear markers on screen 
        clear_objects = menu_handler_.insert("Clear", std::bind(&IntMenu::clearObjects, this, _1));

    }
    //Interactive Actions
    void repairOperation(const MarkerFeedback::ConstSharedPtr &feedback) {

        if (feedback->menu_entry_id == detect_surfaces)
            execute("detect surfaces");
    }

    //moves Callbacks 
    void moveCommand(const MarkerFeedback::ConstSharedPtr &feedback){
        if (feedback->menu_entry_id == move_home){
            execute("move home");
        }
        if (feedback -> menu_entry_id == move_3d_Mouse){
            execute("3D Mouse");
        }
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
    Menu::EntryHandle menu_repair, detect_surfaces, 
                      menu_move, move_home, move_3d_Mouse, move_keyboard,  
                      menu_tools, grinder, p_grinder, h_grinder, vacuum, gripper, marker, 
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