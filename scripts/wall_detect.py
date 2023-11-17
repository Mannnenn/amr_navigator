#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from pcl import VoxelGrid

def callback(data):
    # Convert the point cloud message to a PCL point cloud
    cloud = pc2.read_points(data, skip_nans=True)
    cloud = [[x, y, z] for x, y, z, _ in cloud]
    pcl_cloud = pcl.PointCloud()
    pcl_cloud.from_list(cloud)

    # Apply a voxel grid filter to downsample the point cloud
    voxel_filter = pcl_cloud.make_voxel_grid_filter()
    voxel_filter.set_leaf_size(0.1, 0.1, 0.1)
    downsampled_cloud = voxel_filter.filter()

    # Remove the wall using a statistical outlier removal filter
    sor_filter = downsampled_cloud.make_statistical_outlier_filter()
    sor_filter.set_mean_k(50)
    sor_filter.set_std_dev_mul_thresh(1.0)
    filtered_cloud = sor_filter.filter()

    # Publish the filtered point cloud
    filtered_cloud_msg = pc2.create_cloud(data.header, data.fields, pc2.points_to_xyzrgb(filtered_cloud.to_list()))
    pub.publish(filtered_cloud_msg)

rospy.init_node('point_cloud_filter')
sub = rospy.Subscriber('/livox/lidar', PointCloud2, callback)
pub = rospy.Publisher('/filtered_cloud', PointCloud2, queue_size=10)
rospy.spin()