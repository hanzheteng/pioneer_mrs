#include <ros/ros.h> 
#include <pioneer_mrs/Pioneer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory");
  ros::NodeHandle nh;
  
  Pioneer* pioneer = new Pioneer(nh);
  geometry_msgs::Pose2D pose_hp;
  geometry_msgs::Vector3 vel_hp; 
  ros::Publisher pub_vel_hp = nh.advertise<geometry_msgs::Vector3>("RosAria/cmd_vel_hp", 1);

  const float T = 0.1; // sample period (second)
  const float K = 0.3; // tune K
  const float V = 0.4, W = 0.3; // reference V W (metric)
  const float R = V/W; // radius

  float t = 0; // time (second)
  float x_ref = 0, y_ref = 0, vx_ref = 0, vy_ref = 0; // reference trajectory
  float x_hp = 0, y_hp = 0; // hand point
  float ux = 0, uy = 0; // desired velocity

  ros::Rate rate(10);
  
  while(ros::ok())
  {
    // generate present trajectory
    x_ref = R * cos(W*t);
    y_ref = R * sin(W*t);
    vx_ref = V * cos(W*t);
    vy_ref = V * sin(W*t);
    t = t + T; 
    ROS_DEBUG_STREAM("x_ref="<<x_ref<<"; y_ref="<<y_ref<<"; vx_ref="<<vx_ref<<"; vy_ref="<<vy_ref<<";\n");

    // get present robot position
    pose_hp = pioneer->getPoseHp();
    x_hp = pose_hp.x;
    y_hp = pose_hp.y;

    // generate desired velocity
    ux = vx_ref - K * (x_hp - x_ref);
    uy = vy_ref - K * (y_hp - y_ref);
    ROS_INFO_STREAM("ux="<<ux<<"; uy="<<uy<<";\n");

    // pub vel
    vel_hp.x = ux;
    vel_hp.y = uy;
    vel_hp.z = 0;
    pub_vel_hp.publish(vel_hp);

    ros::spinOnce();
    rate.sleep();
  }

}
