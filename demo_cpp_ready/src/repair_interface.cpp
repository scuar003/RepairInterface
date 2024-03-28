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
        normalize();
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
        poses_subscriber_ = create_subscription<geometry_msgs::msg::PoseArray>("detected_surfaces", 10, std::bind(&RepairInterface::detectSurfacesCallback, this, std::placeholders::_1));
        startMenu();
        renderer_publisher_ = create_publisher<Marker>("visualize_detected_surfaces", 10);
        selector_server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>("selector", shared_from_this());
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
        marker.points.push_back(vector2point(plane.vertex(0)));
        marker.points.push_back(vector2point(plane.vertex(2)));
        marker.points.push_back(vector2point(plane.vertex(3)));
        // Color
        marker.color.r = 1.0f;
        marker.color.g = 0.4f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
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
        // Scale
        marker.scale.x = 1.0f;
        marker.scale.y = 1.0f;
        marker.scale.z = 1.0f;
        // Triangle 1
        marker.points.push_back(vector2point(plane.vertex(0)));
        marker.points.push_back(vector2point(plane.vertex(1)));
        marker.points.push_back(vector2point(plane.vertex(2)));
        // Triangle 2
        marker.points.push_back(vector2point(plane.vertex(0)));
        marker.points.push_back(vector2point(plane.vertex(2)));
        marker.points.push_back(vector2point(plane.vertex(3)));
        // Color
        marker.color.r = 1.0f;
        marker.color.g = 0.4f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
        renderer_publisher_->publish(marker);
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
    void startMenu()
    {
        // first entry
        interactions = menu_handler_.insert("Interactions");
        //grind = menu_handler_.insert(interactions, "Grind", std::bind(&TaskAction::grindCallback, this, _1));
        // Menu::EntryHandle paint = menu_handler_.insert(interactions, "Paint");
        // Menu::EntryHandle vacum = menu_handler_.insert(interactions, "Vacum");

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

        // third entry
        // Menu::EntryHandle selected_surface_submenu = menu_handler_.insert("switch");
        // std::vector<std::string> types{"PLANE", "SPHERE", "CYLINDER", "CUBE"};
        // for (int i = 0; i < 4; ++i)
        // {
        //     std::ostringstream s;
        //     s << types[i];
        //     next_entry = menu_handler_.insert(selected_surface_submenu, s.str(), std::bind(&TaskAction::modeCallback, this, _1));
        //     menu_handler_.setCheckState(next_entry, Menu::UNCHECKED);
        // }

        // menu_handler_.setCheckState(next_entry, Menu::CHECKED);
    }

    Menu::EntryHandle interactions;
    Menu menu_handler_;

    rclcpp::Publisher<Marker>::SharedPtr renderer_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr poses_subscriber_;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> selector_server_;
    std::vector<Plane> m_planes;
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
