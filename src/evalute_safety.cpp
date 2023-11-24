// Subscribe /cmd_raw topic (nav_msgs::twist) and /close_points_num, /course_points_num(std_msgs::Float32).reduce speed as the mount of /close_points_num, /course_points_num.Ratio is 0 to 1.

#include <ros/ros.h>
#include <nav_msgs/Twist.h>
#include <std_msgs/Float32.h>

nav_msgs::Twist cmd_raw;
std_msgs::Float32 close_points_num, course_points_num;

void cmdRawCallback(const nav_msgs::Twist::ConstPtr& msg) {
    cmd_raw = *msg;
}

void closePointsCallback(const std_msgs::Float32::ConstPtr& msg) {
    close_points_num = *msg;
}

void coursePointsCallback(const std_msgs::Float32::ConstPtr& msg) {
    course_points_num = *msg;
}

float calculateSpeedReduction() {
    // Implement your logic here to calculate the speed reduction ratio
    // based on close_points_num and course_points_num
    return 1.0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "safety_evaluator");
    ros::NodeHandle nh;

    ros::Subscriber cmd_raw_sub = nh.subscribe("/cmd_raw", 10, cmdRawCallback);
    ros::Subscriber close_points_sub = nh.subscribe("/close_points_num", 10, closePointsCallback);
    ros::Subscriber course_points_sub = nh.subscribe("/course_points_num", 10, coursePointsCallback);
    ros::Publisher cmd_vel_pub = nh.advertise<nav_msgs::Twist>("/cmd_vel", 10);

    ros::Rate loop_rate(20);
    while (ros::ok()) {
        float reduction_ratio = calculateSpeedReduction();
        cmd_raw.linear.x *= reduction_ratio;
        cmd_raw.angular.z *= reduction_ratio;
        cmd_vel_pub.publish(cmd_raw);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}