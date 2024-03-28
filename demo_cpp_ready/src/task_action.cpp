#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>


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
using Vector = Eigen::Vector3d;

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



class TaskAction : public rclcpp::Node
{
public:
    // constructor
    // initializations
    TaskAction() : Node("TaskAction") {}
    void init()
    {
        server_ = std::make_unique<Server>("Task_server", shared_from_this());
        startMenu();
        renderer_publisher_ = create_publisher<Marker>("visualize_detected_surfaces", 10);
        repair_execute_publisher = create_publisher<geometry_msgs::msg::PoseArray>("repair_area", 10);
        surface_sub = create_subscription<geometry_msgs::msg::PoseArray>("selected_surface",10, std::bind(&TaskAction::poseArrayCallback, this, _1)); 
        selected_area_subscriber = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&TaskAction::selectedAreaCallback, this, _1));
        
    }

private:
    void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
        {
            auto v = poses2vectors(*msg);
            int n = v.size();
            //check if multiple of 4
            if(!n % 4){
                std::cout << "Ops, no plane was selected" <<std::endl;
                return;
            }
            //
            m_planes.clear();
            for (int i = 0; i < n; i +=4){
                m_planes.push_back(Plane(v[i],v[i + 1], v[i + 2], v[i + 3]));
            }
            std::cout << "selected_plane " << std::endl;
            for (long unsigned int i = 0; i < m_planes.size(); ++i)
                createPlane(m_planes[i], i);

        }
   
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
    void createPlane(const Plane &plane, int id) {
        clearMarkers();
        auto marker = makePlane(plane, id);
        auto int_plane = makeMenuPlane("TaskAction", marker);
        // Add the interactive marker to the server
        server_->insert(int_plane);
        menu_handler_.apply(*server_, "TaskAction");
        // Apply changes to the interactive marker server
        server_->applyChanges();
    }
    void clearMarkers() {
        Marker marker;
        marker.header.stamp = rclcpp::Clock().now();
        marker.action = Marker::DELETEALL;
        marker.header.frame_id = "base_link";
        renderer_publisher_->publish(marker);
    }


    // void createPlane(const Marker::SharedPtr marker_msg)
    // {
    //    
    //     //auto marker = makePlane(marker_msg); // Adjust the scale based on the calculated dimensions
    //     auto int_marker = makeMenuPlane("TaskAction", marker_msg);
    //     server_->insert(int_marker);
    //     menu_handler_.apply(*server_, "TaskAction");
    //     server_->applyChanges();
        
    // }
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

    geometry_msgs::msg::Point evectortoPoint(const Vector &eigen_vec)
    {
        geometry_msgs::msg::Point point;
        point.x = eigen_vec.x();
        point.y = eigen_vec.y();
        point.z = eigen_vec.z();
        return point;
    }

    //////////////////////////////////////////////////////////////////////
    void startMenu()
    {
        // first entry
        interactions = menu_handler_.insert("Interactions");
        grind = menu_handler_.insert(interactions, "Grind", std::bind(&TaskAction::grindCallback, this, _1));
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
        Menu::EntryHandle selected_surface_submenu = menu_handler_.insert("switch");
        std::vector<std::string> types{"PLANE", "SPHERE", "CYLINDER", "CUBE"};
        for (int i = 0; i < 4; ++i)
        {
            std::ostringstream s;
            s << types[i];
            next_entry = menu_handler_.insert(selected_surface_submenu, s.str(), std::bind(&TaskAction::modeCallback, this, _1));
            menu_handler_.setCheckState(next_entry, Menu::UNCHECKED);
        }

        menu_handler_.setCheckState(next_entry, Menu::CHECKED);
    }

    void grindCallback(const MarkerFeedback::ConstSharedPtr &feedback)
    {
        
        if (feedback->menu_entry_id == grind)
        {
            geometry_msgs::msg::PoseArray corners;
            corners.header.frame_id = "base_link";
            corners.header.stamp = get_clock()->now();

            for (const auto &point : repair_area_corners)
            {
                geometry_msgs::msg::Pose pose;
                pose.position.x = point.x;
                pose.position.y = point.y;
                pose.position.z = point.z;

                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.0;
                pose.orientation.w = 1.0;

                corners.poses.push_back(pose);
            }
            std::cout << "Publising corners!" << std::endl;
            repair_execute_publisher->publish(corners);
        }
    }

    void modeCallback(const MarkerFeedback::ConstSharedPtr &feedback)
    {
        menu_handler_.setCheckState(next_entry, Menu::UNCHECKED);
        next_entry = feedback->menu_entry_id;
        menu_handler_.setCheckState(next_entry, Menu::CHECKED);

        RCLCPP_INFO(get_logger(), "Switching to menu entry #%d", next_entry);

        menu_handler_.reApply(*server_);
        server_->applyChanges();
    }

    Marker makePlane(const Plane &plane, int id)
    {
        // type, scale, color
        Marker pType;
        pType.type = Marker::TRIANGLE_LIST;
        pType.id = id;
        pType.scale.x = 1.0f;
        pType.scale.y = 1.0f;
        pType.scale.z = 0.01;
        pType.color.r = 0.0f;
        pType.color.g = 1.0f;
        pType.color.b = 0.0f;
        pType.color.a = 1.0;
        pType.pose.orientation.w = 1.0f;

        pType.points.push_back(vector2point(plane.vertex(0)));
        pType.points.push_back(vector2point(plane.vertex(1)));
        pType.points.push_back(vector2point(plane.vertex(2)));
        //
        pType.points.push_back(vector2point(plane.vertex(0)));
        pType.points.push_back(vector2point(plane.vertex(2)));
        pType.points.push_back(vector2point(plane.vertex(3)));

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

    double distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
    {
        return std::sqrt(
            std::pow(p2.x - p1.x, 2) +
            std::pow(p2.y - p1.y, 2) +
            std::pow(p2.z - p1.z, 2));
    }

    // Declarations
    std::unique_ptr<Server> server_;
    Menu menu_handler_;
    Menu::EntryHandle interactions;
    Menu::EntryHandle grind;
    Menu::EntryHandle next_entry;
    ////////////////////////////////////////////////////////////////////////////////////////////////
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr repair_execute_publisher;
    rclcpp::Publisher<Marker>::SharedPtr selected_area_publisher;
    rclcpp::Publisher<Marker>::SharedPtr renderer_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr surface_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr selected_area_subscriber;
    std::vector<geometry_msgs::msg::Point> repair_area;
    std::vector<geometry_msgs::msg::Point> repair_area_corners;
    std::vector<geometry_msgs::msg::Point> s_points;
    std::vector<geometry_msgs::msg::Point> plane_points;
    std::vector<Plane> m_planes;
   


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskAction>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}