#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>



//alias
using namespace std::placeholders; 
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using Rstring = std_msgs::msg::String;


pcl::PointCloud<pcl::PointXYZ>::Ptr msgToPcl(const PointCloud2 &msg) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(msg, *cloud);
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int meank, double stdDevMulThresh) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
    return cloud_filtered;

}



class SurfaceDetection : public rclcpp::Node {
    public:
        SurfaceDetection() : Node("surface_detection") {
            pcl_subscriber = create_subscription<PointCloud2>("camera/depth/color/points", 10,  std::bind(&SurfaceDetection::cameraCallback, this, _1));
            cmd_subscriber = create_subscription<Rstring>("menu_action", 10, std::bind(&SurfaceDetection::cmdCallback, this, _1));
        }


    private:
        void cameraCallback(const PointCloud2::ConstSharedPtr msg) {
            pcl_msg = msg;
        }

        void cmdCallback(const Rstring::ConstSharedPtr msg) {
            if(msg -> data == "detect surfaces") {
                std::cout << "Detecting Surfaces..." << std::endl;
                detectSurfaces();
                std::cout << "...detecting surfaces completed" << std::endl;
            }
        }

        void detectSurfaces() {
            if (!pcl_msg) {
                std::cout << "ups, no pointcloud was detected" << std::endl;
                return;
            }
            auto points = msgToPcl(*pcl_msg);
            points = removeOutliers(points, 100, 1);
        }

    //declarations 
    rclcpp::Subscription<Rstring>::SharedPtr cmd_subscriber;
    rclcpp::Subscription<PointCloud2>::SharedPtr pcl_subscriber;

    PointCloud2::ConstSharedPtr pcl_msg;



};



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    //auto node = std::make_shared<SurfaceDetection>();
    rclcpp::spin(std::make_shared<SurfaceDetection>());
    rclcpp::shutdown();
    return 0;
}