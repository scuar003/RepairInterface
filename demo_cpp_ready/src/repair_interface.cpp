#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <Eigen/Dense>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "interactive_markers/interactive_marker_server.hpp"
#include <interactive_markers/menu_handler.hpp>
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::placeholders; 
using Marker = visualization_msgs::msg::Marker;
using Vector = Eigen::Vector3d;
using Menu = interactive_markers::MenuHandler;
using IntControl = visualization_msgs::msg::InteractiveMarkerControl;
using IntMarker = visualization_msgs::msg::InteractiveMarker;
using MarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;




Vector pose2vector(const geometry_msgs::msg::Pose &pose)
{
    return Vector(pose.position.x, pose.position.y, pose.position.z);
}

geometry_msgs::msg::Point vector2point(const Vector &v)
{
    geometry_msgs::msg::Point p;
    p.x = v[0];
    p.y = v[1];
    p.z = v[2];
    return p;
}

std::vector<Vector> poses2vectors(const geometry_msgs::msg::PoseArray &poses)
{
    std::vector<Vector> v;
    for (const auto &pose : poses.poses)
        v.push_back(pose2vector(pose));
    return v;
}

class Plane
{
public:
    Plane(const Vector &c1, const Vector &c2, const Vector &c3, const Vector &c4) {
        m_corners.resize(4);
        m_corners[0] = c1;
        m_corners[1] = c2;
        m_corners[2] = c3;
        m_corners[3] = c4;
        //normalize();
    }

    std::vector<Vector> corners() const { return m_corners; }

    Vector vertex(int i) const { return m_corners[i]; }

    Vector centroid() const {
        return 0.25*(m_corners[0] + m_corners[1] + m_corners[2] + m_corners[3]);
    }

    Vector normal() const {
        Vector u = m_corners[1] - m_corners[0];
        Vector v = m_corners[2] - m_corners[0];
        Vector n = u.cross(v).normalized();
        return n;
    }

    Vector projectPointOntoPlane(const Vector& point) const {
        Vector planePoint = m_corners[0]; // Can use any point on the plane, here using the first corner
        Vector n = normal(); // Normal of the plane
        double d = (point - planePoint).dot(n);
        return point - d * n; // Projection formula
    }

    Eigen::Quaterniond quaternion() const {
        Vector n = normal();
        Eigen::Quaterniond q;
        q.setFromTwoVectors(Vector::UnitZ(), n);
        return q;
    }

    double length() const {
        return (m_corners[1] - m_corners[0]).norm();
    }

    double width() const {
        return (m_corners[2] - m_corners[1]).norm();
    }

    void normalize() {
        Vector c = centroid();
        Vector n = normal();
        Vector v = (std::abs(n.z()) < 0.9) ? Vector(0, 0, 1) : Vector(1, 0, 0);
        Vector i = n.cross(v).normalized();
        Vector j = n.cross(i).normalized();
        double l = length();
        double w = width();
        m_corners[0] = c + 0.5 * l * i + 0.5 * w * j;
        m_corners[1] = c + 0.5 * l * i - 0.5 * w * j;
        m_corners[2] = c - 0.5 * l * i - 0.5 * w * j;
        m_corners[3] = c - 0.5 * l * i + 0.5 * w * j;
    }

    void updateCorner(int index, const Vector &new_position) {
        if (index >= 0 && index < 4) {
            m_corners[index] = new_position;
        }
    }

    void print() {
        std::cout << "Plane: {";
        for ( const auto &corner : m_corners)
            std::cout << corner << ",";
        std::cout << "}" << std::endl;
    }

private:
    std::vector<Vector> m_corners;
    Vector m_size;
};


class RepairInterface : public rclcpp::Node {
public:

    RepairInterface() : Node("repair_interface") {}

    void init() {
        selector_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("selector", shared_from_this());
        startMenu();
        poses_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>("detected_surfaces", 10, std::bind(&RepairInterface::detectSurfacesCallback, this, std::placeholders::_1));
        renderer_publisher_ = create_publisher<Marker>("visualize_detected_surfaces", 10);
        selected_area_publisher = create_publisher<Marker>("selected_area", 10);
        repair_publishers["grind"] = this->create_publisher<geometry_msgs::msg::PoseArray>("repair_area/grind", 10);
        repair_publishers["expo_marker"] = this->create_publisher<geometry_msgs::msg::PoseArray>("repair_area/expo_marker", 10);
        repair_publishers["vacum"] = this->create_publisher<geometry_msgs::msg::PoseArray>("repair_area/vacum", 10);
    }

    void detectSurfacesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        auto v = poses2vectors(*msg);
        int n = v.size();
        // Check if multiple of 4 corners
        if (!n % 4) {
            std::cout << "Ops, " << n << " not engough plane corners!" << std::endl;
            return;
        }
        // Create planes
        m_planes.clear();
        for (int i = 0; i < n; i += 4)
            m_planes.push_back(Plane(v[i], v[i + 1], v[i + 2], v[i + 3]));
        std::cout << m_planes.size() << " detected!" << std::endl;
        update();
    }

    void update() {
        clearMarkers();
        for (long unsigned int i = 0; i < m_planes.size(); ++i)
            showMarkers(m_planes[i], i);
    }

    void clearMarkers() {
        Marker marker;
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = Marker::DELETEALL;
        marker.header.frame_id = "base_link";
        renderer_publisher_->publish(marker);
    }

    void showMarkers(const Plane &plane, int id) {
        showPlane(plane, id);
        showEdge(plane, id);
        showSelector(plane, id);
        
    }

   

    void createPlane(const Plane &plane, int id) {
        clearMarkers();
        showCorners(plane, id);

        auto marker = makePlane(plane, id);
        auto int_plane = makeMenuPlane("TaskAction", plane);  // Pass the plane object
        // Add the interactive marker to the server
        selector_server_->insert(int_plane);
        menu_handler_.apply(*selector_server_, "TaskAction");
        // Apply changes to the interactive marker server
        selector_server_->applyChanges();
    }


    void showPlane(const Plane &plane, int id) {
        Marker marker;
        marker.type = Marker::TRIANGLE_LIST;
        marker.header.frame_id = "base_link";
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = Marker::ADD;
        marker.ns = "repair_surfaces";
        marker.id = id;
        marker.pose.orientation.w = 1.0f;
        // Scale
        marker.scale.x = 1.0f;
        marker.scale.y = 1.0f;
        marker.scale.z = 1.0f;
        // Triangle 1
        marker.points.push_back(vector2point(plane.vertex(0)));
        marker.points.push_back(vector2point(plane.vertex(1)));
        marker.points.push_back(vector2point(plane.vertex(2)));
        // Triangle 2
        marker.points.push_back(vector2point(plane.vertex(1)));
        marker.points.push_back(vector2point(plane.vertex(3)));
        marker.points.push_back(vector2point(plane.vertex(2)));
        // Color
        marker.color.r = 1.0f;
        marker.color.g = 0.4f;
        marker.color.b = 0.0f;
        marker.color.a = 0.25f;
        renderer_publisher_->publish(marker);
    }

    void showCorners(const Plane &plane, int id) {
        for (int i = 0; i < 4; ++i) {  // Assuming each plane has 4 corners
            IntMarker int_marker;
            int_marker.header.frame_id = "base_link";
            int_marker.header.stamp = rclcpp::Clock().now();
            int_marker.name = "corner_" + std::to_string(id) + "_" + std::to_string(i);
            int_marker.description = "Corner " + std::to_string(i);

            int_marker.pose.position = vector2point(plane.vertex(i));
            int_marker.scale = 0.05; // Adjust the scale of the interactive marker if necessary

            IntControl move_control;
            move_control.interaction_mode = IntControl::MOVE_PLANE;
            move_control.orientation.w = 1;
            move_control.orientation.x = 0;
            move_control.orientation.y = 1;
            move_control.orientation.z = 0;
            
            move_control.always_visible = true;

            Marker marker;
            marker.type = Marker::SPHERE;
            marker.scale.x = 0.05f; // Sphere size
            marker.scale.y = 0.05f;
            marker.scale.z = 0.05f;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;

            move_control.markers.push_back(marker);
            int_marker.controls.push_back(move_control);

            selector_server_->insert(int_marker, std::bind(&RepairInterface::processCornerFeedback, this, _1));
            selector_server_->applyChanges();
        }
    }
    void processCornerFeedback(const MarkerFeedback::ConstSharedPtr &feedback) {
        if (feedback->event_type == MarkerFeedback::MOUSE_UP) {
            std::string marker_name = feedback->marker_name;
            size_t first_underscore = marker_name.find('_');
            size_t last_underscore = marker_name.rfind('_');
            int plane_id = std::stoi(marker_name.substr(first_underscore + 1, last_underscore - first_underscore - 1));
            int corner_id = std::stoi(marker_name.substr(last_underscore + 1));

            Vector new_position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
            m_planes[plane_id].updateCorner(corner_id, new_position);
            m_planes[plane_id];
            update();
        }
    }
   

 
    Marker makePlane(const Plane &plane, int id) {
        Marker marker;
        marker.type = Marker::TRIANGLE_LIST;
        marker.header.frame_id = "base_link";
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = Marker::ADD;
        marker.ns = "repair_surfaces";
        marker.id = id;
        marker.pose.orientation.w = 1.0f;
        // Scale
        marker.scale.x = 1.0f;
        marker.scale.y = 1.0f;
        marker.scale.z = 1.0f;
        // Triangle 1
        marker.points.push_back(vector2point(plane.vertex(0)));
        marker.points.push_back(vector2point(plane.vertex(1)));
        marker.points.push_back(vector2point(plane.vertex(2)));
        // Triangle 2
        marker.points.push_back(vector2point(plane.vertex(1)));
        marker.points.push_back(vector2point(plane.vertex(3)));
        marker.points.push_back(vector2point(plane.vertex(2)));
        // Color
        marker.color.r = 1.0f;
        marker.color.g = 0.4f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7f;
        return marker;
    }
    
    IntControl makePlaneControl(const Marker &p_)
    {
        IntControl planeControl;
        planeControl.always_visible = true;
        planeControl.markers.push_back(p_);
        planeControl.interaction_mode = IntControl::BUTTON;

        return planeControl;
    }
   
    IntMarker makeMenuPlane(const std::string &name, const Plane &plane)
    {
        IntMarker int_marker;
        int_marker.header.frame_id = "base_link";
        int_marker.name = name;
        int_marker.description = "Selected Plane";

        // Set the position of the interactive marker to the centroid of the plane
        auto centroid = plane.centroid();
        int_marker.pose.position.x = centroid.x();
        int_marker.pose.position.y = centroid.y();
        int_marker.pose.position.z = centroid.z();

        // Set the orientation to match the plane's normal
        
        int_marker.scale = 0.5;
        int_marker.pose.orientation.w = 1.0;
        int_marker.pose.orientation.x = 0.0;
        int_marker.pose.orientation.y = 1.0;
        int_marker.pose.orientation.z = 0.0;

        // Add the plane visualization marker
        Marker marker = makePlane(plane, 0);  // Assuming 0 as the id for simplicity
        IntControl plane_control = makePlaneControl(marker);
        int_marker.controls.push_back(plane_control);

        // Add rotational control around the plane's normal axis
        IntControl r_control;
        r_control.orientation_mode = IntControl::FIXED;
        r_control.interaction_mode = IntControl::ROTATE_AXIS;
        r_control.orientation = int_marker.pose.orientation;  // Align the rotational control with the plane's orientation
        r_control.name = "rotate";
        int_marker.controls.push_back(r_control);

        return int_marker;
    }


    void showEdge(const Plane &plane, int id) {
        Marker marker;
        marker.type = Marker::LINE_STRIP;
        marker.header.frame_id = "base_link";
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = Marker::ADD;
        marker.ns = "repair_surfaces";
        marker.id = id + 200;
        marker.pose.orientation.w = 1.0f;
        marker.scale.x = 0.01f;
        // Vectors
        marker.points.push_back(vector2point(plane.vertex(0)));
        marker.points.push_back(vector2point(plane.vertex(1)));
        marker.points.push_back(vector2point(plane.vertex(3)));
        marker.points.push_back(vector2point(plane.vertex(2)));
        marker.points.push_back(vector2point(plane.vertex(0)));
        // Color
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
        renderer_publisher_->publish(marker);
    }
    
    void showSelector(const Plane &plane, int id) {
        // Create an interactive marker
        IntMarker int_marker;
        int_marker.header.frame_id = "base_link";
        int_marker.header.stamp = rclcpp::Clock().now();
        int_marker.name = "repair_surface_" + std::to_string(id);
        int_marker.description = "Surface " + std::to_string(id);

        // Set the position of the interactive marker
        auto centroid = plane.centroid();
        int_marker.pose.position.x = centroid[0];
        int_marker.pose.position.y = centroid[1];
        int_marker.pose.position.z = centroid[2];
        int_marker.scale = 0.1; // Adjust the scale of the interactive marker if necessary

        // Create a sphere control for the interactive marker
        IntControl button_control;
        button_control.interaction_mode = IntControl::BUTTON;
        button_control.name = "button_control";

        // Add a sphere to the button control
        Marker marker;
        marker.type = Marker::SPHERE;
        marker.scale.x = 0.05f; // Sphere size
        marker.scale.y = 0.05f;
        marker.scale.z = 0.05f;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;

        button_control.markers.push_back(marker);
        button_control.always_visible = true; // Make the control always visible

        // Add the control to the interactive marker
        int_marker.controls.push_back(button_control);
        menu_handler_.apply(*selector_server_, "TaskAction");

        // Add the interactive marker to the server
        selector_server_->insert(int_marker, std::bind(&RepairInterface::processFeedback, this, std::placeholders::_1));
        // Apply changes to the interactive marker server
        selector_server_->applyChanges();
    }
   
    void processFeedback(const MarkerFeedback::ConstSharedPtr &feedback) {
        if (feedback->event_type == MarkerFeedback::BUTTON_CLICK) {
            std::cout << "Button for surface " << feedback->marker_name << " pressed." << std::endl;
            // Extract the numeric ID from the marker name assuming it follows the "repair_surface_<id>" format
            std::string selected_id_str = feedback->marker_name.substr(std::string("repair_surface_").length());
            int selected_id = std::stoi(selected_id_str);

            // Clear all interactive markers except the selected one
            for (size_t i = 0; i < m_planes.size(); ++i) {
                std::string marker_name = "repair_surface_" + std::to_string(i);
                if (i != selected_id) {
                    selector_server_->erase(marker_name);
                }
            }

            // Clear all visualization markers except those related to the selected sphere/plane

            clearMarkers();
            createPlane(m_planes[selected_id], selected_id);

        }
    }

    void showPath(const std::vector<Vector> &path, int id)
    {
        Marker marker;
        marker.type = Marker::LINE_STRIP;
        marker.header.frame_id = "base_link";
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = Marker::ADD;
        marker.ns = "repair_surfaces";
        marker.id = id + 4;
        marker.pose.orientation.w = 1.0f;
        marker.scale.x = 0.01f;

        for (const auto &waypoint : path)
            marker.points.push_back(vector2point(waypoint));

        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
        renderer_publisher_->publish(marker);
    }

private:

    void repairArea()
    {
    

        // Create a LINE_STRIP marker to represent the repair area
        Marker line_strip;
        line_strip.header.frame_id = "base_link";
        line_strip.header.stamp = rclcpp::Clock().now();
        line_strip.ns = "repair_area";
        line_strip.id = 0; // Unique ID for this marker
        line_strip.type = Marker::LINE_STRIP;
        line_strip.action = Marker::ADD;

        // LINE_STRIP markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.005; // Specify a suitable line width

        // Set the color of the line strip
        line_strip.color.r = 1.0f;
        line_strip.color.g = 0.0f;
        line_strip.color.b = 0.0f;
        line_strip.color.a = 1.0f; // Don't forget to set the alpha!

        // Assign the points from plane_points to the marker
        for (const auto &point : repair_area)
        {
            line_strip.points.push_back(point);
        }
        // Connect the last point to the first to close the loop
        line_strip.points.push_back(repair_area[0]);

        repair_area_corners = repair_area; 

        // Publish the marker
        selected_area_publisher->publish(line_strip);

        RCLCPP_INFO(this->get_logger(), "Published repair area marker.");
    }
    void startMenu()
    {
        // first entry
        interactions = menu_handler_.insert("Interactions");
        // repair operatiopns 
        grind = menu_handler_.insert(interactions, "grind", std::bind(&RepairInterface::stateCallback, this, _1));
        menu_handler_.setCheckState(grind, Menu::UNCHECKED);

        expo_marker = menu_handler_.insert(interactions, "expo_marker", std::bind(&RepairInterface::stateCallback, this, _1));
        menu_handler_.setCheckState(expo_marker, Menu::UNCHECKED);

        vacum = menu_handler_.insert(interactions, "Vacum", std::bind(&RepairInterface::stateCallback, this, _1));
        menu_handler_.setCheckState(vacum, Menu::UNCHECKED);

        //second entry
        clear_selection = menu_handler_.insert("Clear Selection", std::bind(&RepairInterface::taskClearSelection, this, _1));
        
        // for (auto op : op_names){
        //     menu_operation_handler = menu_handler_.insert(interactions, op, std::bind(&RepairInterface::stateCallback, this, _1));
        //     menu_handler_.setCheckState(menu_operation_handler, Menu::UNCHECKED);
        // }


    }
    
    void stateCallback(const MarkerFeedback::ConstSharedPtr &feedback){
        handle = feedback -> menu_entry_id;
        if(handle == grind) {
            current_task = "grind";
            menu_handler_.setCheckState(grind, Menu::CHECKED);
            menu_handler_.setCheckState(expo_marker, Menu::UNCHECKED);
            menu_handler_.setCheckState(vacum, Menu::UNCHECKED);
            repair_area.clear();
            selected_area_subscriber = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&RepairInterface::taskGrind, this, _1));


        }
        if (handle == expo_marker) {
            current_task = "expo_marker";
            menu_handler_.setCheckState(expo_marker, Menu::CHECKED);
            menu_handler_.setCheckState(grind, Menu::UNCHECKED);
            menu_handler_.setCheckState(vacum, Menu::UNCHECKED);
            repair_area.clear();
            selected_area_subscriber = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&RepairInterface::taskExpoMarker, this, _1));
            
        }
        if (handle == vacum) {
            current_task = "vacum";
            menu_handler_.setCheckState(vacum, Menu::CHECKED);
            menu_handler_.setCheckState(expo_marker, Menu::UNCHECKED);
            menu_handler_.setCheckState(grind, Menu::UNCHECKED); 
            repair_area.clear();
            selected_area_subscriber = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&RepairInterface::taskVacum, this, _1));
           
        }

  
        menu_handler_.reApply(*selector_server_);
        selector_server_ -> applyChanges();
    }



   
    void taskGrind (const geometry_msgs::msg::PointStamped::SharedPtr msg) {

        if (repair_area.size() < 4)
        {
            repair_area.push_back(msg->point);
            RCLCPP_INFO(this->get_logger(), "Point collected: (%.2f, %.2f, %.2f)", msg->point.x, msg->point.y, msg->point.z);
        }
        
        if (repair_area.size() == 4)
        {
            repairArea();
            acceptRepair();
            
            
            } 
    }

    void taskExpoMarker (const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (repair_area.size() < 4)
        {
            repair_area.push_back(msg->point);
            RCLCPP_INFO(this->get_logger(), "Point collected: (%.2f, %.2f, %.2f)", msg->point.x, msg->point.y, msg->point.z);
        }
        
        if (repair_area.size() == 4)
        {
            repairArea();
            acceptRepair();
            
        }

    }

    void taskVacum (const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        repair_area.push_back(msg -> point);
        RCLCPP_INFO(this -> get_logger(), "Point collected:(%.2f, %.2f, %.2f)", msg->point.x, msg->point.y, msg->point.z);
        if (repair_area.size() > 2 ) {
            repairArea();
            
        }
        if (repair_area.size() >= 3) {
            acceptRepair();
        }
    }

    void acceptRepair() {
        // Define an interactive marker
        IntMarker execute_marker;
        execute_marker.header.frame_id = "base_link";
        execute_marker.header.stamp = rclcpp::Clock().now();
        execute_marker.name = "Execute_Repair";
        execute_marker.description = "Press to execute repair";

        // Set the position (adjust according to your needs)
        execute_marker.pose.position.x = 0.25; // Example positions
        execute_marker.pose.position.y = 0.25;
        execute_marker.pose.position.z = 0.25;
        execute_marker.scale = 0.1; // Scale of the interactive marker

        // Create a control that will act as a button
        IntControl button_control;
        button_control.interaction_mode = IntControl::BUTTON;
        button_control.name = "button";

        // Create a marker for the button
        Marker marker;
        marker.type = Marker::CUBE;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Add the marker to the button control
        button_control.markers.push_back(marker);
        button_control.always_visible = true;

        // Add the control to the interactive marker
        execute_marker.controls.push_back(button_control);

        // Insert the interactive marker into the server and apply changes
        std::cout << "The executer button has been Published" << std::endl;
        selector_server_->insert(execute_marker, std::bind(&RepairInterface::waitToExecute, this, std::placeholders::_1));
        selector_server_->applyChanges();
    }


     void waitToExecute(const MarkerFeedback::ConstSharedPtr &feedback) {
        if (feedback -> event_type == MarkerFeedback::BUTTON_CLICK) {
            auto execute_it = repair_publishers.find(current_task);
            if(execute_it != repair_publishers.end()){
                auto& publisher = execute_it -> second;
                geometry_msgs::msg::PoseArray corners;
                corners.header.frame_id = "base_link";
                corners.header.stamp = get_clock() -> now();
                for (const auto &point : repair_area_corners){
                    geometry_msgs::msg::Pose pose;
                    pose.position.x = point.x;
                    pose.position.y = point.y;
                    pose.position.z = point.z;

                    pose.orientation.x = 0.0f;
                    pose.orientation.y = 0.0f;
                    pose.orientation.z = 0.0f;
                    pose.orientation.w = 1.0f;

                    corners.poses.push_back(pose);
                }
                std::cout << "Publishing corners to " << current_task << "!" << std::endl;
                publisher->publish(corners);
            }
        } else {
            std::cerr << "No publisher available for task: " << current_task << std::endl;
        }
     }



     
    //second menu entry
    void taskClearSelection (const MarkerFeedback::ConstSharedPtr &feedback) {
  
        Marker clear_marker;
        clear_marker.header.frame_id = "base_link"; // Same frame_id as the original marker
        clear_marker.header.stamp = rclcpp::Clock().now();
        clear_marker.ns = "repair_area"; // Same namespace as the original marker
        clear_marker.id = 0; // Same ID as the original marker
        clear_marker.action = Marker::DELETE; // Action set to DELETE

        // Publish the clear_marker to remove the previously published marker
        selected_area_publisher->publish(clear_marker);

        RCLCPP_INFO(this->get_logger(), "Repair area marker cleared.");

        repair_area.clear();
    }
    



    //declarations
    Menu menu_handler_; //
    Menu::EntryHandle interactions; // first entry
    Menu::EntryHandle grind, expo_marker, vacum; //possible operations  
    Menu::EntryHandle clear_selection;
    Menu::EntryHandle handle;
    Menu::EntryHandle menu_operation_handler;

    rclcpp::Publisher<Marker>::SharedPtr renderer_publisher_;
    rclcpp::Publisher<Marker>::SharedPtr selected_area_publisher;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> selector_server_;


    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poses_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr selected_area_subscriber;
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> repair_publishers;

    std::vector<Plane> m_planes;
    std::vector<geometry_msgs::msg::Point> repair_area;
    std::vector<geometry_msgs::msg::Point> repair_area_corners;

    std::string current_task;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RepairInterface>();
    node -> init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
