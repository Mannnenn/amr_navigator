// Subscribe /cmd_raw topic (geometry_msgs::twist) and /close_points_num, /course_points_num(std_msgs::Float32).reduce speed as the mount of /close_points_num, /course_points_num.Ratio is 0 to 1.

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

geometry_msgs::Twist cmd_raw, cmd_vel;
int close_lim, course_lim;
int close_points_num, course_points_num;

void getParams() {
    ros::NodeHandle nh;
    if (!nh.getParam("close_lim", close_lim)) {
        ROS_WARN("Failed to get param 'close_lim'");
    }
    if (!nh.getParam("course_lim", course_lim)) {
        ROS_WARN("Failed to get param 'course_lim'");
    }
}

void cmdRawCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    cmd_raw = *msg;
}

void closePointsCallback(const std_msgs::Float32::ConstPtr& msg) {
    std_msgs::Int32 int_msg;
    int_msg.data = static_cast<int>(msg->data);
    close_points_num = int_msg.data;
}

void coursePointsCallback(const std_msgs::Float32::ConstPtr& msg) {
    std_msgs::Int32 int_msg;
    int_msg.data = static_cast<int>(msg->data);
    course_points_num = int_msg.data;
}

float calculateSpeedReduction(int close_points_num, int course_points_num) {
    int close_lim = 50; // define your limit for close points
    int course_lim = 10; // define your limit for course points

    if (close_points_num > close_lim) {
        return 0;
    } else if (course_points_num > course_lim) {
        return 0.5;
    } else {
        return 1.0;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "safety_evaluator");
    ros::NodeHandle nh;

    getParams();
    ros::Subscriber cmd_raw_sub = nh.subscribe("/cmd_raw", 10, cmdRawCallback);
    ros::Subscriber close_points_sub = nh.subscribe("/close_points_num", 10, closePointsCallback);
    ros::Subscriber course_points_sub = nh.subscribe("/course_points_num", 10, coursePointsCallback);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate loop_rate(20);
    while (ros::ok()) {
        float reduction_ratio = calculateSpeedReduction(close_points_num, course_points_num);
        cmd_vel.linear.x = cmd_raw.linear.x * reduction_ratio;
        cmd_vel.angular.z = cmd_raw.angular.z * reduction_ratio;
        cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}