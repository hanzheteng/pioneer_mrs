#include <ros/ros.h> 
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <math.h>
/*
 * The generation of trajectories needs to be centralized
 * because different agents may have their own time clock.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_cmd");
  ros::NodeHandle nh;
  ros::Publisher pub[5];
  pub[0] = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/robot1/trajectory", 1);
  pub[1] = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/robot2/trajectory", 1);
  pub[2] = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/robot3/trajectory", 1);
  pub[3] = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/robot4/trajectory", 1);
  pub[4] = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/robot5/trajectory", 1);

  trajectory_msgs::JointTrajectoryPoint point[5];
  for(int i=0; i<=4; i++)
  {
    point[i].positions.resize(2);
    point[i].velocities.resize(2);
  }

  const double T = 0.1; // sample period (second)
  const double V = 0.4, W = 0.3; // reference V W (metric)
  const double R = V/W; // radius
  const double phase = 6.28/5;  // (2*pi) / 5 robots
  double x_ref = 0, y_ref = 0, vx_ref = 0, vy_ref = 0;

  double t = 0; // time (second)
  ros::Rate rate(1/T);

  while(ros::ok())
  {
    for(int i=0; i<=4; i++)
    {
      // generate present trajectory
      x_ref = R * cos(W*t + i*phase);
      y_ref = R * sin(W*t + i*phase);
      vx_ref = V * cos(W*t + i*phase);
      vy_ref = V * sin(W*t + i*phase);
      ROS_DEBUG_STREAM("robot"<<i+1<<": x_ref="<<x_ref<<"; y_ref="<<y_ref<<"; vx_ref="<<vx_ref<<"; vy_ref="<<vy_ref<<";\n");

      point[i].positions[0] = x_ref;
      point[i].positions[1] = y_ref;
      point[i].velocities[0] = vx_ref;
      point[i].velocities[1] = vy_ref;
      pub[i].publish(point[i]);
    }
    t = t + T; 
    rate.sleep();
  }
  
}
