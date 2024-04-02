#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <Eigen/Dense>

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
        // Calculate perpendicular vectors on the plane
        Vector v = (std::abs(n.z()) < 0.9) ? Vector(0, 0, 1) : Vector(1, 0, 0);
        Vector i = n.cross(v).normalized();
        Vector j = n.cross(i).normalized();
        // Sides
        double l = length();
        double w = width();
        // calculate four corners
        m_corners[0] = c + 0.5 * l * i + 0.5 * w * j;
        m_corners[1] = c + 0.5 * l * i - 0.5 * w * j;
        m_corners[2] = c - 0.5 * l * i - 0.5 * w * j;
        m_corners[3] = c - 0.5 * l * i + 0.5 * w * j;
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
        selected_area_subscriber = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&RepairInterface::selectedAreaCallback, this, _1));
        renderer_publisher_ = create_publisher<Marker>("visualize_detected_surfaces", 10);
        selected_area_publisher = create_publisher<Marker>("selected_area", 10);
        repair_execute_publisher = create_publisher<geometry_msgs::msg::PoseArray>("repair_area", 10);
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
        auto int_plane = makeMenuPlane("TaskAction", marker);
        // Add the interactive marker to the server
        selector_server_->insert(int_plane);
        menu_handler_.apply(*selector_server_, "TaskAction");
        // Apply changes to the interactive marker server
        selector_server_->applyChanges();
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
        marker.points.push_back(vector2point(plane.vertex(2)));
        marker.points.push_back(vector2point(plane.vertex(3)));
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
    IntMarker makeMenuPlane(const std::string &name, const Marker &p_)
    {
        IntMarker plane;
        plane.header.frame_id = "base_link";
        plane.name = name;
        plane.description = "Selected Plane";

        plane.controls.push_back(makePlaneControl(p_));

        // free move

        // rotational controls
        //  IntControl rControl;
        //  rControl.orientation_mode = IntControl::VIEW_FACING;
        //  rControl.interaction_mode = IntControl::ROTATE_AXIS;
        //  rControl.orientation.w = 1.0;
        //  rControl.name = "rotate";
        //  plane.controls.push_back(rControl);

        // IntControl mControl;
        // mControl.orientation_mode = IntControl::VIEW_FACING;
        // mControl.interaction_mode = IntControl::MOVE_PLANE;
        // mControl.independent_marker_orientation = true;
        // mControl.name = "move";
        // mControl.markers.push_back(p_);
        // mControl.always_visible = true;
        // plane.controls.push_back(mControl);

        return plane;
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
;
        
   
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
        marker.points.push_back(vector2point(plane.vertex(2)));
        marker.points.push_back(vector2point(plane.vertex(3)));
        // Color
        
        marker.color.r = 1.0f;
        marker.color.g = 0.4f;
        marker.color.b = 0.0f;
        marker.color.a = 0.7f;
        renderer_publisher_->publish(marker);
    }
    void showCorners(const Plane &plane, int id) {
    for (int i = 0; i < 4; ++i) {  // Assuming each plane has 4 corners
        Marker marker;
        marker.type = Marker::SPHERE;
        marker.header.frame_id = "base_link";
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = Marker::ADD;
        marker.ns = "repair_corners";  // Different namespace for corners
        marker.id = id * 10 + i;  // Ensure unique ID for each corner across planes
        marker.pose.position = vector2point(plane.vertex(i));
        
        // Set the orientation of the sphere to identity, as it doesn't matter for spheres
        marker.pose.orientation.w = 1.0;

        // Scale represents the size of the sphere
        marker.scale.x = 0.05f;  // Sphere diameter in meters
        marker.scale.y = 0.05f;
        marker.scale.z = 0.05f;

        // Color of the sphere
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;  // Alpha value of 1 means the sphere is not transparent

        renderer_publisher_->publish(marker);
    }
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
        marker.points.push_back(vector2point(plane.vertex(2)));
        marker.points.push_back(vector2point(plane.vertex(3)));
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
            selectedPlane(selected_id);

        }
    }

    void selectedPlane(int selected_id) {
        // Now creates the marker related to the selected plane
        clearMarkers();
        createPlane(m_planes[selected_id], selected_id);
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
    void selectedAreaCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Collect points until we have four
        if (repair_area.size() < 4)
        {
            repair_area.push_back(msg->point);
            RCLCPP_INFO(this->get_logger(), "Point collected: (%.2f, %.2f, %.2f)", msg->point.x, msg->point.y, msg->point.z);
        }

        // When four points are collected, draw the line strip and then clear the points for a new area
        if (repair_area.size() == 4)
        {
            repairArea();
            repair_area.clear(); // Ready to collect new points for another area
        }
    }
    void repairArea()
    {
        // Check if we have exactly four points
        if (repair_area.size() != 4)
        {
            RCLCPP_ERROR(this->get_logger(), "Expecting exactly four points to form the repair area.");
            return;
        }

        // Create a LINE_STRIP marker to represent the repair area
        Marker line_strip;
        line_strip.header.frame_id = "base_link";
        line_strip.header.stamp = rclcpp::Clock().now();
        line_strip.ns = "repair_area";
        line_strip.id = 0; // Unique ID for this marker
        line_strip.type = Marker::LINE_STRIP;
        line_strip.action = Marker::ADD;

        // LINE_STRIP markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.01; // Specify a suitable line width

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
        grind = menu_handler_.insert(interactions, "Grind", std::bind(&RepairInterface::taskGrind, this, _1));
        paint = menu_handler_.insert(interactions, "Paint", std::bind(&RepairInterface::taskPaint, this, _1));
        vacum = menu_handler_.insert(interactions, "Vacum", std::bind(&RepairInterface::taskVacum, this, _1));

        //second entry
        clear_selection = menu_handler_.insert("Clear Selection", std::bind(&RepairInterface::taskClearSelection, this, _1));
        // next steps
        // Menu::EntryHandle whole_plane = menu_handler_.insert(grind, "whole_plane");
        // Menu::EntryHandle select_area = menu_handler_.insert(grind, "select_area");

        // second entry
        // Menu::EntryHandle details = menu_handler_.insert("Details");

        // part of the example code
        //  entry = menu_handler_.insert(entry, "sub");
        //  entry = menu_handler_.insert(entry, "menu", [this](const MarkerFeedback::ConstSharedPtr)
        //                                                                                          {
        //                                                                                              RCLCPP_INFO(get_logger(), "the menu has been found");
        //                                                                                          });
        // menu_handler_.setCheckState(menu_handler_.insert("something", std::bind(&TaskAction::enableCallback, this, _1)), Menu::CHECKED);

    }
    //First menu entry
    void taskGrind (const MarkerFeedback::ConstSharedPtr &feedback) {

        if(feedback -> menu_entry_id == grind) {
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
            std::cout << "Publishing corners!" << std::endl; 
            repair_execute_publisher->publish(corners);
        
        }
    }

    void taskPaint (const MarkerFeedback::ConstSharedPtr &feedback) {}

    void taskVacum (const MarkerFeedback::ConstSharedPtr &feedback) {}
    
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
    Menu::EntryHandle grind, paint, vacum; //possible operations  
    Menu::EntryHandle clear_selection;
    
    rclcpp::Publisher<Marker>::SharedPtr renderer_publisher_;
    rclcpp::Publisher<Marker>::SharedPtr selected_area_publisher;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> selector_server_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poses_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr selected_area_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr repair_execute_publisher;


    std::vector<Plane> m_planes;
    std::vector<geometry_msgs::msg::Point> repair_area;
    std::vector<geometry_msgs::msg::Point> repair_area_corners;

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
