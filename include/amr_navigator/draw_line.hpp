// draw_line.hpp
#ifndef DRAW_LINE_HPP
#define DRAW_LINE_HPP

#include <ros/ros.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <vector>

class PolygonPublisher {
public:
    PolygonPublisher();
    void make_polygonarray(double linear_velocity, double turning_radius, double width);
    const double width;
    const double ratio;
    double turning_radius;
    double linear_velocity;

private:
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle nh;

    void callback(const geometry_msgs::Twist::ConstPtr& msg);
    std::vector<geometry_msgs::Point32> define_half_circle(double linear_velocity, double turning_radius, double width, int num_points);
    std::vector<geometry_msgs::Point32> define_rectangle(double linear_velocity, double width);

};

#endif // DRAW_LINE_HPP