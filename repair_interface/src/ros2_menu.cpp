#include "repair_interface/ros2_menu.h"

MainMenu::MainMenu() : Node ("Main_Menu") {}

void MainMenu::init () {
     interactive_server_ = std::make_unique<IntServer>("menu_server", shared_from_this());
     startMenu();
     auto marker = makeMarker(1.0);
     interactive_server_ -> insert(makeMarkerMenu("IntMenu", marker));
     menu_handler.apply(*interactive_server_, "IntMenu");
     interactive_server_ -> applyChanges();

     command_pub = create_publisher<StringRos2>("menu_action", 10);
}

void MainMenu::execute(std::string cmd) {
    StringRos2 msg;
    msg.data = cmd;
    command_pub -> publish(msg);
}

void MainMenu::startMenu() {
    //Actions - Interactions
        menu_repair = menu_handler.insert("Repair Operations");
        detect_surfaces = menu_handler.insert(menu_repair, "Detect Surfaces", std::bind(&MainMenu::menuAction, this, _1));
        scan_env = menu_handler.insert(menu_repair, "Scan Environment(SOON)", std::bind(&MainMenu::menuAction, this, _1));

        //move commands 
        menu_move = menu_handler.insert("Move");
        move_home = menu_handler.insert(menu_move, "Home", std::bind(&MainMenu::menuAction, this, _1));
        move_3d_Mouse = menu_handler.insert(menu_move, "3D Mouse", std::bind(&MainMenu::menuAction, this, _1));
        move_keyboard = menu_handler.insert(menu_move, "Keyboard(SOON)", std::bind(&MainMenu::menuAction, this, _1));
 
        //Tools  
        menu_tools = menu_handler.insert("Get/Change Tools(SOON)");
        grinder = menu_handler.insert(menu_tools, "Grinders");
        p_grinder = menu_handler.insert(grinder, "P-Grinder", std::bind(&MainMenu::menuAction, this, _1));
        h_grinder = menu_handler.insert(grinder, "H-Grinder", std::bind(&MainMenu::menuAction, this, _1));
        vacuum = menu_handler.insert(menu_tools, "Vacuum", std::bind(&MainMenu::menuAction, this, _1));
        gripper = menu_handler.insert(menu_tools, "Gripper", std::bind(&MainMenu::menuAction, this, _1));
        marker = menu_handler.insert(menu_tools, "Marker", std::bind(&MainMenu::menuAction, this, _1));

        //Dashboard options 
        // op4 = menu_handler_.insert("Soon...");
}

void MainMenu::menuAction (const MarkerFeedback::ConstSharedPtr &feedback) {

    //Operations
        if (feedback->menu_entry_id == detect_surfaces)
            execute("detect surfaces");
        if(feedback -> menu_entry_id == scan_env)
            execute("scan env");
    
    //moves 
        if (feedback->menu_entry_id == move_home)
            execute("move home");
        if (feedback -> menu_entry_id == move_3d_Mouse)
            execute("3D Mouse");
        if(feedback -> menu_entry_id == move_keyboard)
            execute("Keyboard");
        
    //Get tools 
        if(feedback -> menu_entry_id == p_grinder)
            execute("p_grinder");
        if(feedback -> menu_entry_id == h_grinder)
            execute("h_grinder");
        if(feedback -> menu_entry_id == vacuum)
            execute("vacuum");
        if(feedback -> menu_entry_id == gripper)
            execute("gripper");
        if(feedback -> menu_entry_id == marker)
            execute("marker");

    //soon
}

Marker MainMenu::makeMarker(const float scale) {
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

IntControl MainMenu::makeMarkerControl(const Marker &p) {
    IntControl menuControl;
    menuControl.always_visible = true;
    menuControl.markers.push_back(p);
    menuControl.interaction_mode = IntControl::BUTTON;

    return menuControl;
}

IntMarker MainMenu::makeMarkerMenu (const std::string &name, const Marker &p_) {
    IntMarker menu;
    menu.header.frame_id = "base_link";
    menu.pose.position.x = 1.0;
    menu.pose.position.y = 1.0;
    menu.pose.position.z = 1.0;
    menu.name = name;
    menu.description = "Menu";
    menu.scale = 0.25;

    menu.controls.push_back(makeMarkerControl(p_));

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


