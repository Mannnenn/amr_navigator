#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pcd_pub;
bool is_saved = false;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Apply voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud.makeShared());
    sor.setLeafSize(0.01f, 0.01f, 0.01f); // Set the size of the voxel grid
    sor.filter(cloud);

    // Get the PCD file path from parameter
    std::string pcd_file_path;
    if (!ros::param::get("pcd_file_path", pcd_file_path)) {
        ROS_ERROR("Failed to get param 'pcd_file_path'");
        return;
    }

    // Save to PCD file only once
    if (!is_saved) {
        pcl::io::savePCDFileASCII(pcd_file_path, cloud);
        is_saved = true;
    }

    // Publish the PCD file (optional)
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    pcd_pub.publish(output);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;


    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/livox_lidar", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pcd_pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // Spin
    ros::spin();
}