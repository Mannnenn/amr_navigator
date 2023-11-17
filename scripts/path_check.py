#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, OccupancyGrid
from costmap_2d import Costmap2D
from geometry_msgs.msg import PoseStamped

# Initialize the ROS node
rospy.init_node('path_checker')

# Create a Costmap2D object
costmap = Costmap2D()

def costmap_callback(costmap_msg):
    global costmap
    # Update the costmap with the new data
    costmap = Costmap2D(costmap_msg)

def path_callback(path):
    for pose_stamped in path.poses:
        # Convert the pose to the costmap's frame
        pose_in_costmap_frame = convert_pose_to_costmap_frame(pose_stamped.pose)

        # Get the cost of the cell that the pose is in
        cost = costmap.getCost(pose_in_costmap_frame.position.x, pose_in_costmap_frame.position.y)

        # Check if the cost is greater than a certain threshold
        if cost > 50:  # Change this value based on your needs
            rospy.loginfo('The path intersects with an obstacle')

# Subscribe to the costmap and path topics
rospy.Subscriber('/my_cost_map', OccupancyGrid, costmap_callback)
rospy.Subscriber('/waypoint_path', Path, path_callback)

# Spin the ROS node so that it keeps listening for new messages
rospy.spin()