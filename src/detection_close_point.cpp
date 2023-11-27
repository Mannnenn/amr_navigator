#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/distances.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>

#include <cmath>


ros::Publisher pub_close_points;
ros::Publisher pub_close_points_num;

double linear_velocity = 0;
double turning_radius = 0;


void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs::PointCloud2 data to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input,*cloud_input);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_input);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter (*cloud_input);

    // Create the PassThrough filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    // Set the axis to filter along
    pass.setFilterFieldName("z");
    // Set the range of acceptable values
    pass.setFilterLimits(0.0, 2.0);
    // Set the input cloud
    pass.setInputCloud(cloud_input);
    pass.filter(*cloud_input);

    const double width = 0.5;
    const double ratio = 3.0;

    // Create a new point cloud for the filtered points
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //If robot turn, consider to ring area
    if (turning_radius != 0)
    {
        double center_angle;
        center_angle = (linear_velocity * ratio) / turning_radius;

        pcl::CropBox<pcl::PointXYZ> boxFilter;

        if (turning_radius > 0) {
            // Set min and max for positive turning_radius
            boxFilter.setMin(Eigen::Vector4f(0, -width/2, 0.1, 1.0));
            boxFilter.setMax(Eigen::Vector4f(turning_radius * std::sin(center_angle), turning_radius, 2.0, 1.0));
        }
        else if (turning_radius < 0){
            // Set min and max for negative turning_radius
            boxFilter.setMin(Eigen::Vector4f(0, turning_radius, 0.1, 1.0));
            boxFilter.setMax(Eigen::Vector4f(turning_radius * std::sin(center_angle), width/2, 2.0, 1.0));
        }
        boxFilter.setInputCloud(cloud_input);
        boxFilter.filter(*cloud_input);

        double inner_radius = abs(turning_radius) - width/2; // Set your desired distance
        double outer_radius = abs(turning_radius) + width/2; // Set your desired distance


        // Iterate over the points in the point cloud
        for (const auto& point : cloud_input->points)
        {
            // Compute the Euclidean distance from the point to the turning radius center
            double point_distance = std::sqrt(point.x * point.x + (point.y - turning_radius) * (point.y - turning_radius));

            // If the distance is less than or equal to the desired distance, add the point to the filtered cloud
            if (point_distance <= outer_radius && point_distance >= inner_radius)
            {
                filtered_cloud->points.push_back(point);
            }
        }
    }

    //If robot go straight, consider to box area
    else
    {
        // Create the filtering object
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(0.0, -width/2, 0.0, 1.0));
        boxFilter.setMax(Eigen::Vector4f(linear_velocity*ratio, width/2, 2.0, 1.0));
        boxFilter.setInputCloud(cloud_input);
        boxFilter.filter(*filtered_cloud);

    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = true;

    //Publish point cloud which is removed background
    sensor_msgs::PointCloud2 output;

    pcl::toROSMsg (*filtered_cloud, output);
    output.header.frame_id = "livox_frame";
    pub_close_points.publish (output);

    size_t num_points = filtered_cloud->points.size();
    std_msgs::Float32 msg;
    msg.data = num_points;
    pub_close_points_num.publish(msg);
}

void callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    turning_radius = msg->data[0];
    linear_velocity = msg->data[1];
}



int main (int argc, char** argv)
{
    ros::init (argc, argv, "pcl_change_detector");
    ros::NodeHandle nh;

    pub_close_points = nh.advertise<sensor_msgs::PointCloud2>("close_points", 10);
    pub_close_points_num = nh.advertise<std_msgs::Float32>("close_points_num", 10);

    ros::Subscriber sub_points = nh.subscribe ("/livox/lidar", 10, cloudCB);
    ros::Subscriber sub = nh.subscribe("radius_vel", 10, callback);
    ros::spin ();

    return 0;
}