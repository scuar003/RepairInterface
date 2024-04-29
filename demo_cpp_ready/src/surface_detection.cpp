#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <algorithm> 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h> 

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>



//alias
using namespace std::placeholders; 
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Rstring = std_msgs::msg::String;
using PoseArray = geometry_msgs::msg::PoseArray;


pcl::PointCloud<pcl::PointXYZ>::Ptr msgToPcl(const PointCloud2 &msg) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(msg, *cloud);
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int meank, double stdDevMulThresh) {
    //Statical Outlier Removal
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
    return cloud_filtered;

}
std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> detectPlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // Voxel downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f); // Voxel size equivalent to Python's g_voxel_size
    sor.filter(*cloud_filtered);

    // Normal estimation
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_filtered);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.1); // Radius equivalent to Python's radius parameter
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*cloud_normals);

    // Planar segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);

    // Assume multiple planes detection
    std::vector<std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> planes;
    while (cloud_filtered->points.size() > 0.1 * cloud->points.size()) {
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>());
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*plane);
        planes.push_back(plane);

        // Remove the planar inliers from the input cloud
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
    }

    return planes; // Returns a vector of point clouds, each representing a detected plane
}
struct PlaneOfInterest {
    Eigen::Vector3d center;
    Eigen::Matrix3d R;
    Eigen::Vector3d extent;
};

std::vector<Eigen::Vector3d> getCornersOfPlaneOfInterest(const PlaneOfInterest& plane) {
    Eigen::Vector3d center = plane.center;
    Eigen::Matrix3d R = plane.R;
    Eigen::Vector3d extents = plane.extent;

    // The plane's normal is assumed to be along the axis with the smallest extent
    int min_extent_idx = 0;
    double min_extent = extents[0];
    for (int i = 1; i < 3; ++i) {
        if (extents[i] < min_extent) {
            min_extent = extents[i];
            min_extent_idx = i;
        }
    }

    std::vector<int> indices = {0, 1, 2};
    indices.erase(indices.begin() + min_extent_idx);

    // Compute the four corners of the plane
    std::vector<Eigen::Vector3d> corners;
    for (int i : {-1, 1}) {
        for (int j : {-1, 1}) {
            Eigen::Vector3d corner_vect = Eigen::Vector3d::Zero();
            corner_vect[indices[0]] = extents[indices[0]] / 2 * i;
            corner_vect[indices[1]] = extents[indices[1]] / 2 * j;
            // Ensure the corners are on the plane by making the component along the normal very small or zero
            corner_vect[min_extent_idx] = 0;  // Place the corner on the plane, assuming the plane is at the center
            Eigen::Vector3d corner = center + R * corner_vect;
            corners.push_back(corner);
        }
    }

    return corners;
}

class SurfaceDetection : public rclcpp::Node {
public:
    SurfaceDetection() : Node("surface_detection"), tf_buffer_(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME)) {
       initializeSubscribersAndPublishers();
    }
    void initialize() {
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_, shared_from_this(), false);
    }

private:
    void initializeSubscribersAndPublishers() {
        pcl_subscriber = create_subscription<PointCloud2>("camera/depth/color/points", 10, std::bind(&SurfaceDetection::cameraCallback, this, _1));
        cmd_subscriber = create_subscription<Rstring>("menu_action", 10, std::bind(&SurfaceDetection::cmdCallback, this, _1));
        publisher_planes = create_publisher<PoseArray>("detected_surfaces", 10);
    }

    void cameraCallback(const PointCloud2::ConstSharedPtr msg) {
        pcl_msg = msg;
    }

    void cmdCallback(const Rstring::ConstSharedPtr msg) {
        if (msg->data == "detect surfaces") {
            std::cout << "Detecting Surfaces..." << std::endl;
            detectSurfaces();
            std::cout << "...detecting surfaces completed" << std::endl;
        }
    }

    void detectSurfaces() {
    if (!pcl_msg) {
        std::cout << "Ups, no pointcloud was detected" << std::endl;
        return;
    }
    auto points = msgToPcl(*pcl_msg);
    points = removeOutliers(points, 100, 1);
    auto planes = detectPlanes(points);
    if (planes.empty()) {
        std::cout << "No planes detected." << std::endl;
        return;
    }
    std::vector<std::vector<Eigen::Vector3d>> corners;
    for (const auto& plane : planes) {
        PlaneOfInterest poi;
        // Example initialization, replace with actual data extraction from plane
        poi.center = Eigen::Vector3d::Zero(); // Set actual values
        poi.R = Eigen::Matrix3d::Identity();  // Set actual values
        poi.extent = Eigen::Vector3d::Ones(); // Set actual values

        auto plane_corners = getCornersOfPlaneOfInterest(poi);
        if (plane_corners.empty()) {
            std::cout << "No corners computed for a plane." << std::endl;
        } else {
            corners.push_back(plane_corners);
        }
    }
    if (corners.empty()) {
        std::cout << "No corners were computed for any plane." << std::endl;
    } else {
        publishCorners(corners);
    }
}

    void publishCorners(const std::vector<std::vector<Eigen::Vector3d>>& corners) {
    std::string target_frame = "base_link";
    auto message = geometry_msgs::msg::PoseArray();
    message.header.frame_id = target_frame;
    message.header.stamp = this->get_clock()->now();

    for (const auto& plane_corners : corners) {
        for (const auto& point : plane_corners) {
            auto pose_stamped = geometry_msgs::msg::PointStamped();
            pose_stamped.header.frame_id = "camera_depth_optical_frame";
            pose_stamped.header.stamp = this->get_clock()->now();
            pose_stamped.point.x = point[0];
            pose_stamped.point.y = point[1];
            pose_stamped.point.z = point[2];

            try {
                auto transform = tf_buffer_.lookupTransform(target_frame, pose_stamped.header.frame_id, tf2::TimePointZero);
                geometry_msgs::msg::PointStamped transformed_pose;
                tf2::doTransform(pose_stamped, transformed_pose, transform);

                auto pose = geometry_msgs::msg::Pose();
                pose.position.x = transformed_pose.point.x;
                pose.position.y = transformed_pose.point.y;
                pose.position.z = transformed_pose.point.z;
                pose.orientation.w = 1.0; // Identity quaternion
                message.poses.push_back(pose);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), "Failed to transform point: %s", ex.what());
            }
        }
    }

    if (message.poses.empty()) {
        std::cout << "No transformed poses were added to the message." << std::endl;
    }

    publisher_planes->publish(message);
    std::cout << "Published " << message.poses.size() << " poses." << std::endl;
}

    tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_; 
    rclcpp::Subscription<Rstring>::SharedPtr cmd_subscriber;
    rclcpp::Subscription<PointCloud2>::SharedPtr pcl_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_planes;
    PointCloud2::ConstSharedPtr pcl_msg;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SurfaceDetection>();
    node->initialize();  // Ensure this is called only after the node is fully constructed
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}