#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

using namespace std::chrono_literals;
using namespace Eigen;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;

class PlaneFinder : public rclcpp::Node
{
public:
    PlaneFinder() : Node("plane_finder")
    {
        pcl_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 10, std::bind(&PlaneFinder::camera_callback, this, std::placeholders::_1));
        command_subscriber_ = create_subscription<std_msgs::msg::String>(
            "menu_action", 10, std::bind(&PlaneFinder::command_callback, this, std::placeholders::_1));
        publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("detected_surfaces", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void camera_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pointcloud_msg_ = msg;
    }

    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "detect surfaces")
        {
            RCLCPP_INFO(this->get_logger(), "Detecting surfaces...");
            detect_surfaces();
            RCLCPP_INFO(this->get_logger(), "...detecting surfaces completed!");
        }
    }

    void detect_surfaces()
    {
        if (!pointcloud_msg_)
        {
            RCLCPP_ERROR(this->get_logger(), "No pointcloud data received!");
            return;
        }

        auto cloud = msg_to_pcl(pointcloud_msg_);
        auto planes = detect_planes(cloud);
        publish_planes(planes);
    }

private:
    PointCloudPtr msg_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr &msg)
    {
        PointCloudPtr cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *cloud);
        return cloud;
    }

    std::vector<geometry_msgs::msg::Pose> detect_planes(PointCloudPtr cloud)
    {
        std::vector<geometry_msgs::msg::Pose> planes;

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*cloud);

        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.1);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.compute(*cloud_normals);

        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);
        seg.setInputNormals(cloud_normals);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.segment(*inliers, *coefficients);

        // Calculate plane center and orientation
        Vector3d plane_center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Quaterniond orientation(coefficients->values[6], coefficients->values[7], coefficients->values[8], coefficients->values[9]);

        // Convert the coefficients to poses
        for (auto index : inliers->indices)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = cloud->points[index].x;
            pose.position.y = cloud->points[index].y;
            pose.position.z = cloud->points[index].z;
            planes.push_back(pose);
        }

        return planes;
    }

    // Vector3d calculatePlaneCenter(const pcl::ModelCoefficients::Ptr &coefficients)
    // {
    //     double x = coefficients->values[0];
    //     double y = coefficients->values[1];
    //     double z = coefficients->values[2];
    //     return Vector3d(x, y, z);
    // }
    struct PlaneOfInterest
    {
        Eigen::Vector3d center;
        Eigen::Matrix3d R;
        Eigen::Vector3d extent;
    };

    // Function to get the center of the plane
    Eigen::Vector3d getCenterOfPlaneOfInterest(const PlaneOfInterest &plane)
    {
        return plane.center;
    }

    // Function to get the corners of the plane
    std::vector<Eigen::Vector3d> getCornersOfPlaneOfInterest(const PlaneOfInterest &plane)
    {
        Eigen::Vector3d center = getCenterOfPlaneOfInterest(plane);
        Eigen::Matrix3d R = plane.R;
        Eigen::Vector3d extents = plane.extent;

        // Find the index of the smallest extent to assume it's along the plane's normal
        int min_extent_idx = 0;
        double min_extent_value = extents[0];
        for (int i = 1; i < 3; ++i)
        {
            if (extents[i] < min_extent_value)
            {
                min_extent_idx = i;
                min_extent_value = extents[i];
            }
        }

        // Compute the corners of the plane
        std::vector<Eigen::Vector3d> corners;
        std::vector<int> indices = {0, 1, 2};
        indices.erase(indices.begin() + min_extent_idx); // Remove the index with the smallest extent

        for (int i : {-1, 1})
        {
            for (int j : {-1, 1})
            {
                Eigen::Vector3d corner_vect = Eigen::Vector3d::Zero();
                corner_vect[indices[0]] = extents[indices[0]] / 2 * i;
                corner_vect[indices[1]] = extents[indices[1]] / 2 * j;
                corner_vect[min_extent_idx] = 0; // Ensure the corner is on the plane
                Eigen::Vector3d corner = center + R * corner_vect;
                corners.push_back(corner);
            }
        }

        return corners;
    }

    void publish_planes(const std::vector<geometry_msgs::msg::Pose> &planes)
    {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.frame_id = "base_link";

        for (auto &pose : planes)
        {
            // Transform pose to the 'base_link' frame before publishing
            geometry_msgs::msg::PoseStamped stamped_in, stamped_out;
            stamped_in.header.frame_id = "camera_depth_optical_frame";
            stamped_in.pose = pose;

            try
            {
                auto transform = tf_buffer_->lookupTransform("base_link", "camera_depth_optical_frame", stamped_in.header.stamp);
                tf2::doTransform(stamped_in, stamped_out, transform);
                pose_array.poses.push_back(stamped_out.pose);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to transform plane pose: %s", ex.what());
            }
        }

        publisher_->publish(pose_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg_;
    double plane_length;
    double plane_width;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlaneFinder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}