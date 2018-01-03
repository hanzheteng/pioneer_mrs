#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <pioneer_mrs/Pioneer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer");
  ros::NodeHandle nh;
  Pioneer* pioneer = new Pioneer(nh);
  ros::spin();
}
