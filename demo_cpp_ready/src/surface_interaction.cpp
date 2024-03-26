#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

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
using Evector = Eigen::Vector3d;

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
        repair_execute_publisher = create_publisher<geometry_msgs::msg::PoseArray>("repair_area_corners", 10);
        selected_surface_sub = create_subscription<geometry_msgs::msg::PoseArray>("repair_area", 10, std::bind(&TaskAction::poseArrayCallback, this, _1));
        selected_area_publisher = create_publisher<Marker>("corner_publisher", 10);
        selected_area_subscriber = create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&TaskAction::selectedAreaCallback, this, _1));
    }

private:
    void poseArrayCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        for (const auto &pose : msg->poses)
            s_points.push_back(pose.position);

        if (s_points.size() == 4)
        {
            createPlane();
            s_points.clear(); // Clear points for next plane
            // plane_points.clear();
        }
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

    void createPlane()
    {
        if (s_points.size() != 4)
        {
            RCLCPP_ERROR(this->get_logger(), "Insufficient points to form a plane");
            return;
        }

        Evector centroid(0, 0, 0);
        for (const auto &point : s_points)
        {
            centroid += Evector(point.x, point.y, point.z);
        }
        centroid /= s_points.size();

        // Compute normal vector
        Evector vec1 = Evector(s_points[1].x, s_points[1].y, s_points[1].z) - Evector(s_points[0].x, s_points[0].y, s_points[0].z);
        Evector vec2 = Evector(s_points[2].x, s_points[2].y, s_points[2].z) - Evector(s_points[0].x, s_points[0].y, s_points[0].z);
        Evector normal = vec1.cross(vec2).normalized();

        // Calculate perpendicular vectors on the plane
        Evector arbitraryVec = (std::abs(normal.z()) < 0.9) ? Evector(0, 0, 1) : Evector(1, 0, 0);
        Evector vecX = normal.cross(arbitraryVec).normalized();
        Evector vecY = normal.cross(vecX).normalized();

        Eigen::Quaterniond q;
        q.setFromTwoVectors(Evector::UnitZ(), normal);

        double length = distance(s_points[0], s_points[1]);
        double width = distance(s_points[1], s_points[2]);

        // vec1.normalize();
        // vec2.normalize();
        Evector halflengthVec = vecY * (length / 2.0);
        Evector halfwidthVec = vecX * (width / 2.0);

        // calculate four corners
        Evector corner1 = centroid + halflengthVec + halfwidthVec;
        Evector corner2 = centroid + halflengthVec - halfwidthVec;
        Evector corner3 = centroid - halflengthVec - halfwidthVec;
        Evector corner4 = centroid - halflengthVec + halfwidthVec;

        // send points

        plane_points.push_back(evectortoPoint(corner1));
        plane_points.push_back(evectortoPoint(corner2));
        plane_points.push_back(evectortoPoint(corner3));
        plane_points.push_back(evectortoPoint(corner4));

        // use to test
        RCLCPP_INFO(this->get_logger(), "Corner 1: (%.2f, %.2f, %.2f)", corner1.x(), corner1.y(), corner1.z());
        RCLCPP_INFO(this->get_logger(), "Corner 2: (%.2f, %.2f, %.2f)", corner2.x(), corner2.y(), corner2.z());
        RCLCPP_INFO(this->get_logger(), "Corner 3: (%.2f, %.2f, %.2f)", corner3.x(), corner3.y(), corner3.z());
        RCLCPP_INFO(this->get_logger(), "Corner 4: (%.2f, %.2f, %.2f)", corner4.x(), corner4.y(), corner4.z());

        // use to test

        if (plane_points.size() > 3)
            plane_points = std::vector<geometry_msgs::msg::Point>(plane_points.end() - 4, plane_points.end());

        auto marker = makePlane(width, length); // Adjust the scale based on the calculated dimensions
        marker.pose.position.x = centroid.x();
        marker.pose.position.y = centroid.y();
        marker.pose.position.z = centroid.z();
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w(); // Adjust the scale as needed

        auto int_marker = makeMenuPlane("TaskAction", marker);
        server_->insert(int_marker);
        menu_handler_.apply(*server_, "TaskAction");
        server_->applyChanges();
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
        line_strip.scale.x = 0.02; // Specify a suitable line width

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

    geometry_msgs::msg::Point evectortoPoint(const Evector &eigen_vec)
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

    Marker makePlane(double length, double width)
    {
        Marker pType;
        pType.type = Marker::CUBE;
        // type, scale, color
        pType.scale.x = length;
        pType.scale.y = width;
        pType.scale.z = 0.01;
        pType.color.r = 0.0f;
        pType.color.g = 1.0f;
        pType.color.b = 0.0f;
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
        plane.name = name;
        plane.description = "Interactive Plane";
        plane.scale = 1.0;

        plane.controls.push_back(makePlaneControl(p_));

        // free move

        // rotational controls
        //  IntControl rControl;
        //  rControl.orientation_mode = IntControl::VIEW_FACING;
        //  rControl.interaction_mode = IntControl::ROTATE_AXIS;
        //  rControl.orientation.w = 1.0;
        //  rControl.name = "rotate";
        //  plane.controls.push_back(rControl);

        IntControl mControl;
        mControl.orientation_mode = IntControl::VIEW_FACING;
        mControl.interaction_mode = IntControl::MOVE_PLANE;
        mControl.independent_marker_orientation = true;
        mControl.name = "move";
        mControl.markers.push_back(p_);
        mControl.always_visible = true;
        plane.controls.push_back(mControl);

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
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr selected_surface_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr selected_area_subscriber;
    std::vector<geometry_msgs::msg::Point> repair_area;
    std::vector<geometry_msgs::msg::Point> repair_area_corners;
    std::vector<geometry_msgs::msg::Point> s_points;
    std::vector<geometry_msgs::msg::Point> plane_points;
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