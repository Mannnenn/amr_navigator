#!/usr/bin/python3
# -*- coding: utf-8 -*-

import math
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Polygon, PolygonStamped, Point32, Twist
from jsk_recognition_msgs.msg import PolygonArray


turning_radius = 0
linear_velocity = 0
width = 0.5
ratio = 1.0


def define_half_circle(linear_velocity, turning_radius, width, num_points):
    # Define the center of the circles and the radii
    cx, cy, cz = -turning_radius, 0, 0  # center of the circles, (x,y,z)
    r1, r2 = turning_radius - width/2, turning_radius + width/2  # radii

    # Define the start angle (in radians)
    start_angle = 0  # 0 degrees
    end_angle = (linear_velocity*ratio) / turning_radius
    # Calculate the points along the outer half circle
    points = []
    for i in range(num_points + 1):
        # Calculate the angle (from start_angle to end_angle)
        a = start_angle + (end_angle - start_angle) * i / num_points

        # Calculate the x and y coordinates
        x = cx + r2 * math.cos(a)
        y = cy + r2 * math.sin(a)

        # Create a Point32 message and add it to the list
        points.append(Point32(x=x, y=y, z=cz))

    # Calculate the points along the inner half circle in reverse order
    for i in range(num_points, -1, -1):
        # Calculate the angle (from start_angle to end_angle)
        a = start_angle + (end_angle - start_angle) * i / num_points

        # Calculate the x and y coordinates
        x = cx + r1 * math.cos(a)
        y = cy + r1 * math.sin(a)

        # Create a Point32 message and add it to the list
        points.append(Point32(x=x, y=y, z=cz))

    return points

def define_rectangle(linear_velocity, width):
    # Define the corners of the rectangle
    points = [Point32(x=-width/2, y=0, z=0),  # Bottom left corner
              Point32(x=width/2, y=0, z=0),  # Bottom right corner
              Point32(x=width/2, y=linear_velocity*ratio, z=0),  # Top right corner
              Point32(x=-width/2, y=linear_velocity*ratio, z=0)]  # Top left corner
    return points

def make_polygonarray(linear_velocity, turning_radius, width):
    msg = PolygonArray()
    if turning_radius != 0:
        points = define_half_circle(linear_velocity, turning_radius, width, 20) # Define the corners of the rectangle
    else:
        points = define_rectangle(linear_velocity, width) # Define the corners of the rectangle

    # Create a PolygonStamped message
    p = PolygonStamped()
    p.header.frame_id = "line_link"
    p.polygon.points = points

    # Add the polygon to the PolygonArray message
    msg = PolygonArray()
    msg.header.frame_id = "line_link"
    msg.polygons = [p]

    # Publish the PolygonArray message
    pub.publish(msg)




def callback(msg):
    # Initialize the Twist message
    twist = Twist()
    global turning_radius
    global linear_velocity
    twist = msg

    # Calculate the turning radius
    if twist.angular.z != 0:
        turning_radius = twist.linear.x / twist.angular.z
    else:
        turning_radius = 0

    linear_velocity = twist.linear.x

    linear_msg = Float32()
    linear_msg.data = linear_velocity
    pub_linear.publish(linear_msg)
    radius_msg = Float32()
    radius_msg.data = turning_radius
    pub_radius.publish(radius_msg)



def main():
    rospy.init_node('pub')

    global turning_radius
    global linear_velocity
    global width
    global ratio
    width = 0.5
    ratio = rospy.get_param('line_ratio',10.0) # if faster, the ratio should be smaller

    # Subscribe to the Twist topic
    rospy.Subscriber('cmd_vel', Twist, callback)
    rate = rospy.Rate(25) # 10Hz
    while not rospy.is_shutdown():
        make_polygonarray(linear_velocity, turning_radius, width)
        rate.sleep()
    rospy.spin()

pub = rospy.Publisher('course', PolygonArray, queue_size=1)
pub_linear = rospy.Publisher('linear_velocity', Float32, queue_size=10)
pub_radius = rospy.Publisher('turning_radius', Float32, queue_size=10)

if __name__ == '__main__':
    main()