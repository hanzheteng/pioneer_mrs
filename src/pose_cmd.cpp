#include <ros/ros.h> 
#include <pioneer_mrs/Pioneer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_cmd");
  ros::NodeHandle nh;
  
  Pioneer* pioneer = new Pioneer(nh);
  geometry_msgs::Pose2D diff;
  geometry_msgs::Vector3 vel_hp; 
  ros::Publisher pub_vel_hp = nh.advertise<geometry_msgs::Vector3>("RosAria/cmd_vel_hp", 1);
    
  vel_hp.x = 0;
  vel_hp.y = 0;
  vel_hp.z = 0;

  ros::Rate rate(10);
  
  while(ros::ok())
  {
    diff = pioneer->getPoseDiff();
    if(diff.x > 0.2)
      vel_hp.x = 0.2;
    else
      vel_hp.x = 0;
    if(diff.y > 0.2)
      vel_hp.y = 0.2;
    else
      vel_hp.y = 0;

    ROS_INFO_STREAM(diff);
    pub_vel_hp.publish(vel_hp);

    ros::spinOnce();
    rate.sleep();
  }

}
