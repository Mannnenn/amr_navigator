#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

ros::Publisher array_pub; // Declare this at a global scope or as a class member

void callback(const geometry_msgs::Twist::ConstPtr& msg) {
    geometry_msgs::Twist twist = *msg;
    std_msgs::Float32MultiArray array;
    double turning_radius, linear_velocity;
    if (twist.angular.z != 0) {
        turning_radius = twist.linear.x / twist.angular.z;
        linear_velocity = twist.linear.x;
    } else {
        turning_radius = 0;
        linear_velocity = twist.linear.x;
    }
    array.data.clear();
    array.data.push_back(turning_radius);
    array.data.push_back(linear_velocity);
    array_pub.publish(array);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "array_publisher");
    ros::NodeHandle nh;
    array_pub = nh.advertise<std_msgs::Float32MultiArray>("radius_vel", 10);
    ros::Subscriber sub = nh.subscribe("cmd_raw", 10, callback);
    ros::spin();
    return 0;
}