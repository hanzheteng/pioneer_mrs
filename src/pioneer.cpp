#include <pioneer_mrs/pioneer.h>

Pioneer::Pioneer(ros::NodeHandle& nh, ros::NodeHandle& nh_private):
  HOSTNUM(0),
  HOSTNAME(""), 
  POSE(""),
  HANDPOINT_OFFSET(0)
{
  nh_private.param( "hostname", HOSTNAME, std::string("robot1") );
  ROS_INFO( "[%s]: node launched.", HOSTNAME.c_str() );

  HOSTNUM = std::stoi( HOSTNAME.substr(5) ) - 1; // array index

  nh_private.param( "pose", POSE, std::string("odom") );
  if(POSE != "odom")
    ROS_INFO_STREAM( "[" << HOSTNAME << "]: set pose = " << POSE << " (localization by " << POSE << ")");

  nh_private.param( "handpoint_offset", HANDPOINT_OFFSET, 0.25);
  if(HANDPOINT_OFFSET != 0.25)
    ROS_INFO_STREAM( "[" << HOSTNAME << "]: set handpoint offset = " << HANDPOINT_OFFSET);

  if(POSE == "vicon")
    vicon_pose_sub = nh.subscribe("/vicon/" + HOSTNAME + "/" + HOSTNAME, 1, &Pioneer::viconPoseCallBack, this);
  else 
    odom_pose_sub = nh.subscribe("RosAria/pose", 1, &Pioneer::odomPoseCallBack, this);
}


Pioneer::~Pioneer()
{
  ROS_INFO_STREAM("[" << HOSTNAME << "]: Quitting... \n");
}


void Pioneer::odomPoseCallBack(const nav_msgs::Odometry& msg)
{
  // transform quaternion to Euler angle
  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  this->pose_hp.theta = yaw; //yaw from -3.14 to 3.14, left side is the positive number

  // convert center pose to handpoint pose
  geometry_msgs::Pose2D pose; // center pose
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;
  this->pose_hp.x = pose.x + HANDPOINT_OFFSET * cos(pose.theta);
  this->pose_hp.y = pose.y + HANDPOINT_OFFSET * sin(pose.theta);
  ROS_DEBUG_STREAM("odom: theta="<<pose_hp.theta<<"; x_hp="<<pose_hp.x<<"; y_hp="<<pose_hp.y<<";\n");
}


void Pioneer::viconPoseCallBack(const geometry_msgs::TransformStamped& msg)
{
  // transform quaternion to Euler angle
  tf::Quaternion q(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  this->pose_hp.theta = yaw;
  this->pose_hp.x = msg.transform.translation.x;
  this->pose_hp.y = msg.transform.translation.y;
  ROS_DEBUG_STREAM("vicon: theta="<<pose_hp.theta<<"; x_hp="<<pose_hp.x<<"; y_hp="<<pose_hp.y<<";\n");
}


