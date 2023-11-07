#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>



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

geometry_msgs::Pose convertPoseToCostmapFrame(const geometry_msgs::Pose& pose, tf2_ros::Buffer& tfBuffer) {
    geometry_msgs::Pose pose_in_costmap_frame;

    try {
        // The transform from the path's frame to the costmap's frame
        geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform("costmap_frame", "path_frame", ros::Time(0));

        // Apply the transform to the pose
        tf2::doTransform(pose, pose_in_costmap_frame, transform);
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }

    return pose_in_costmap_frame;
}



void pathCallback(const nav_msgs::Path::ConstPtr& msg, tf2_ros::Buffer& tfBuffer) {
    if (costmap == nullptr) {
        ROS_ERROR("Costmap is null");
        return;
    }
    for (const auto& pose_stamped : msg->poses) {
        // Convert the pose to the costmap's frame
        auto pose_in_costmap_frame = convertPoseToCostmapFrame(pose_stamped.pose, tfBuffer);

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

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Subscriber costmap_sub = nh.subscribe("/costamp_load/my_costmap/costmap", 1000, boost::bind(costmapCallback, _1));
    ros::Subscriber path_sub = nh.subscribe("/waypoints", 1000, boost::bind(pathCallback, _1, boost::ref(tfBuffer)));

    ros::spin();

    return 0;
}