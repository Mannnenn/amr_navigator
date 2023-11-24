#include "amr_navigator/draw_line.hpp"

PolygonPublisher::PolygonPublisher() : width(0.5), ratio(4.0), turning_radius(0), linear_velocity(0) {
    pub = nh.advertise<jsk_recognition_msgs::PolygonArray>("course", 1);
    sub = nh.subscribe("cmd_vel", 1000, &PolygonPublisher::callback, this);
}

void PolygonPublisher::callback(const geometry_msgs::Twist::ConstPtr& msg) {
    geometry_msgs::Twist twist = *msg;
    if (twist.angular.z != 0) {
        turning_radius = twist.linear.x / twist.angular.z;
        linear_velocity = twist.linear.x;
    } else {
        turning_radius = 0;
        linear_velocity = twist.linear.x;
    }
}

std::vector<geometry_msgs::Point32> PolygonPublisher::define_half_circle(double linear_velocity, double turning_radius, double width, int num_points) {
    // Define the center of the circles and the radii
    double cx = 0, cy = -turning_radius, cz = 0;  // center of the circles, (x,y,z)
    double r1 = turning_radius - width/2, r2 = turning_radius + width/2;  // radii

    // Define the start angle (in radians)
    double start_angle = 0;  // 0 degrees
    double end_angle = (linear_velocity*ratio) / turning_radius;
    // Calculate the points along the outer half circle
    std::vector<geometry_msgs::Point32> points;
    for (int i = 0; i <= num_points; i++) {
        // Calculate the angle (from start_angle to end_angle)
        double a = start_angle + (end_angle - start_angle) * i / num_points;

        // Calculate the x and y coordinates
        geometry_msgs::Point32 circle_point;
        circle_point.x = cx + r2 * sin(a);
        circle_point.y = -(cy + r2 * cos(a));
        circle_point.z = cz;

        // Create a Point32 message and add it to the list
        points.push_back(circle_point);
    }

    // Calculate the points along the inner half circle in reverse order
    for (int i = num_points; i >= 0; i--) {
        // Calculate the angle (from start_angle to end_angle)
        double a = start_angle + (end_angle - start_angle) * i / num_points;

        // Calculate the x and y coordinates
        geometry_msgs::Point32 circle_point;
        circle_point.x = cx + r1 * sin(a);
        circle_point.y = -(cy + r1 * cos(a));
        circle_point.z = cz;

        // Create a Point32 message and add it to the list
        points.push_back(circle_point);
    }

    return points;
}

std::vector<geometry_msgs::Point32> PolygonPublisher::define_rectangle(double linear_velocity, double width) {
    std::vector<geometry_msgs::Point32> points;
    geometry_msgs::Point32 point;

    point.x = 0;
    point.y = -width/2;
    point.z = 0;
    points.push_back(point);  // Bottom left corner

    point.y = width/2;
    points.push_back(point);  // Bottom right corner

    point.x = linear_velocity*ratio;
    points.push_back(point);  // Top right corner

    point.y = -width/2;
    points.push_back(point);  // Top left corner

    return points;
};


void PolygonPublisher::make_polygonarray(double linear_velocity, double turning_radius, double width) {
    jsk_recognition_msgs::PolygonArray msg;
    std::vector<geometry_msgs::Point32> points;
    if (turning_radius != 0) {
        points = define_half_circle(linear_velocity, turning_radius, width, 20); // Define the corners of the rectangle
    } else {
        points = define_rectangle(linear_velocity, width); // Define the corners of the rectangle
    }

    // Create a PolygonStamped message
    geometry_msgs::PolygonStamped p;
    p.header.frame_id = "base_link";
    p.polygon.points = points;

    // Add the polygon to the PolygonArray message
    msg.header.frame_id = "base_link";
    msg.polygons.push_back(p);

    // Publish the PolygonArray message
    pub.publish(msg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "pub");
    PolygonPublisher pp;
    ros::Rate rate(25);  // 25Hz

    while (ros::ok()) {
        pp.make_polygonarray(pp.linear_velocity, pp.turning_radius, pp.width);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}