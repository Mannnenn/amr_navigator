#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "costmap");
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tf(tfBuffer);
  costmap_2d::Costmap2DROS* costmap_ros = new costmap_2d::Costmap2DROS("my_costmap", tfBuffer);
  ros::NodeHandle n;
  costmap_ros->start();
  ros::spin();
  return 0;
}