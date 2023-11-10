#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>




costmap_2d::Costmap2D* costmap;

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    // Create a new Costmap2D object
    costmap = new costmap_2d::Costmap2D(msg->info.width, msg->info.height, msg->info.resolution, msg->info.origin.position.x, msg->info.origin.position.y);

    // Fill the costmap with data from the OccupancyGrid message
    for (unsigned int i = 0; i < msg->info.width; i++) {
        for (unsigned int j = 0; j < msg->info.height; j++) {
            unsigned int index = i + j * msg->info.width;
            costmap->setCost(i, j, msg->data[index]);
        }
    }
}

geometry_msgs::Pose convertPoseToCostmapFrame(const geometry_msgs::Pose& pose) {
    // For simplicity, this function assumes that the path's frame and the costmap's frame are in the same coordinate system.
    // If this is not the case, you will need to use a tf listener to transform the pose to the costmap's frame.
    return pose;
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (costmap == nullptr) {
        ROS_ERROR("Costmap is null");
        return;
    }
    for (const auto& pose_stamped : msg->poses) {
        // Convert the pose to the costmap's frame
        auto pose_in_costmap_frame = convertPoseToCostmapFrame(pose_stamped.pose);

        // Get the cost of the cell that the pose is in
        auto cost = costmap->getCost(pose_in_costmap_frame.position.x, pose_in_costmap_frame.position.y);

        // Check if the cost is greater than a certain threshold
        if (cost > 99) {  // Change this value based on your needs
            ROS_INFO("The path intersects with an obstacle");
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_checker");

    ros::NodeHandle nh;

    ros::Subscriber costmap_sub = nh.subscribe("/costamp_load/my_costmap/costmap", 1000, costmapCallback);
    ros::Subscriber path_sub = nh.subscribe("/waypoints", 1000, pathCallback);

    ros::spin();

    return 0;
}