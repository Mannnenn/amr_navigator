#ifndef ODOM_TO_POSE_HPP
#define ODOM_TO_POSE_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pose_pub;

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg);

#endif // ODOM_TO_POSE_HPP