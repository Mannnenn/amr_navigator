#include "amr_navigator/odom_to_pose.hpp"


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header = odom_msg->header;
    pose_stamped_msg.pose = odom_msg->pose.pose;
    pose_pub.publish(pose_stamped_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_pose");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odom_callback);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/pose", 10);

    ros::spin();

    return 0;
}