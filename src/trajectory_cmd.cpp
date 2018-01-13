#include <ros/ros.h> 
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory");
  ros::NodeHandle nh;
  trajectory_msgs::JointTrajectoryPoint point;
  ros::Publisher pub_traj = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("trajectory", 1);

  point.positions.resize(2);
  point.velocities.resize(2);

  const float T = 0.1; // sample period (second)
  const float V = 0.4, W = 0.3; // reference V W (metric)
  const float R = V/W; // radius
  float x_ref = 0, y_ref = 0, vx_ref = 0, vy_ref = 0;

  float t = 0; // time (second)
  ros::Rate rate(10);

  while(ros::ok())
  {
    // generate present trajectory
    x_ref = R * cos(W*t);
    y_ref = R * sin(W*t);
    vx_ref = V * cos(W*t);
    vy_ref = V * sin(W*t);
    ROS_DEBUG_STREAM("x_ref="<<x_ref<<"; y_ref="<<y_ref<<"; vx_ref="<<vx_ref<<"; vy_ref="<<vy_ref<<";\n");

    point.positions[0] = x_ref;
    point.positions[1] = y_ref;
    point.velocities[0] = vx_ref;
    point.velocities[1] = vy_ref;
    pub_traj.publish(point);

    t = t + T; 
    rate.sleep();
  }

}
