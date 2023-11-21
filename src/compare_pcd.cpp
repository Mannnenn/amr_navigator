#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>
#include <pcl/common/distances.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl/filters/passthrough.h>
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sensor (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudDiff (new pcl::PointCloud<pcl::PointXYZ>);
pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (0.25f);

ros::Publisher pub_diff_points;
ros::Publisher pub_course_points;

double linear_velocity = 0;
double turning_radius = 0;


//x, y, z coordinates
double x;
double y;
double z;

//quaternion
double qx;
double qy;
double qz;
double qw;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Get the x, y, z coordinates
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    // Get the quaternion
    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;
    qw = msg->pose.pose.orientation.w;
}

void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{

    // Create a tf listener
    tf::TransformListener listener;

    // Create a string for the target frame
    std::string target_frame = "pcl_link";

    // Wait for the transform to become available
    listener.waitForTransform(target_frame, input->header.frame_id, ros::Time(0), ros::Duration(3.0));

    // Create a new PointCloud2 message for the transformed cloud
    sensor_msgs::PointCloud2 transformed_cloud;

    // Transform the cloud
    pcl_ros::transformPointCloud(target_frame, *input, transformed_cloud, listener);

    // Create a PointCloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_topic(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert the sensor_msgs::PointCloud2 data to pcl::PointCloud
    pcl::fromROSMsg(transformed_cloud, *cloud_topic);


    // The translation (x, y, z)
    Eigen::Translation3f tl(x, y, z);

    // The rotation (qx, qy, qz, qw)
    Eigen::Quaternionf qt(qx, qy, qz, qw);

    // The transformation
    Eigen::Affine3f transform = tl * qt;

    // Apply the transformation to the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr odom_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_topic, *odom_cloud, transform);





    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (odom_cloud);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*odom_cloud);




    octree.setInputCloud (cloud_map);
    octree.addPointsFromInputCloud ();
    octree.switchBuffers ();  // Switch to cloud_sensor

    octree.setInputCloud (odom_cloud);
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;
    octree.getPointIndicesFromNewVoxels (newPointIdxVector);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < newPointIdxVector.size (); ++i)
        cloud_diff->points.push_back(odom_cloud->points[newPointIdxVector[i]]);

    cloud_diff->width = cloud_diff->points.size();
    cloud_diff->height = 1;
    cloud_diff->is_dense = true;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*cloud_diff, output);
    output.header.frame_id = "map";
    pub_diff_points.publish (output);

    const double width = 0.5;
    const double ratio = 4.0;

    if (turning_radius != 0)
    {
        double center_angle;
        center_angle = (linear_velocity * ratio) / turning_radius;

        pcl::CropBox<pcl::PointXYZ> boxFilter;

        if (turning_radius > 0) {
            // Set min and max for positive turning_radius
            boxFilter.setMin(Eigen::Vector4f(0, -width/2, -0.1, 1.0));
            boxFilter.setMax(Eigen::Vector4f(turning_radius * std::sin(center_angle), turning_radius, 2.0, 1.0));
        }
        else if (turning_radius < 0){
            // Set min and max for negative turning_radius
            boxFilter.setMin(Eigen::Vector4f(0, turning_radius, -0.1, 1.0));
            boxFilter.setMax(Eigen::Vector4f(turning_radius * std::sin(center_angle), width/2, 2.0, 1.0));
        }
        boxFilter.setInputCloud(cloud_diff);
        boxFilter.filter(*cloud_diff);

        double inner_radius = abs(turning_radius) - width/2; // Set your desired distance
        double outer_radius = abs(turning_radius) + width/2; // Set your desired distance

        // Create a new point cloud for the filtered points
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Iterate over the points in the point cloud
        for (const auto& point : cloud_diff->points)
        {
            // Compute the Euclidean distance from the point to the turning radius center
            double radius = std::sqrt(point.x * point.x + (point.y - turning_radius) * (point.y - turning_radius));

            // If the distance is less than or equal to the desired distance, add the point to the filtered cloud
            if (point_distance <= outer_radius && point_distance >= inner_radius)
            {
                filtered_cloud->points.push_back(point);
            }
        }

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = "map";
        pub_course_points.publish(output);
    }

    else
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_diff);

        // Create the filtering object
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(0.0, -width/2, 0.0, 1.0));
        boxFilter.setMax(Eigen::Vector4f(linear_velocity*ratio, width/2, 2.0, 1.0));
        boxFilter.setInputCloud(cloud_diff);
        boxFilter.filter(*cloud_diff);

        pass.filter (*cloud_diff);
        pcl::toROSMsg (*cloud_diff, output);
        output.header.frame_id = "map";
        pub_course_points.publish (output);
    }


    octree.switchBuffers ();  // Switch to cloud_sensor

}

void linearCB(const std_msgs::Float32::ConstPtr& msg)
{
    linear_velocity = msg->data;
}

void turningCB(const std_msgs::Float32::ConstPtr& msg)
{
    turning_radius = msg->data;
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_change_detector");
    ros::NodeHandle nh;


    std::string pcd_file_path;
    if (!ros::param::get("pcd_file_path", pcd_file_path)) {
        ROS_ERROR("Failed to get param pcd_file_path");
        return (-1);
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file_path, *cloud_map) == -1)
    {
        PCL_ERROR ("Couldn't read the file cloud_map.pcd \n");
        return (-1);
    }

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_map);
    sor.setLeafSize (0.15f, 0.15f, 0.15f);
    sor.filter (*cloud_map);


    pub_diff_points = nh.advertise<sensor_msgs::PointCloud2>("diff_points", 1);
    pub_course_points = nh.advertise<sensor_msgs::PointCloud2>("course_points", 1);
    ros::Subscriber sub_points = nh.subscribe ("/livox/lidar", 1, cloudCB);
    ros::Subscriber sub_linear = nh.subscribe ("/linear_velocity", 1, linearCB);
    ros::Subscriber sub_turning = nh.subscribe ("/turning_radius", 1, turningCB);
    ros::Subscriber sub = nh.subscribe("/odom", 1000, odomCallback);
    ros::spin ();

    return 0;
}