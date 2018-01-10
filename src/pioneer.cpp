#include <ros/ros.h> 
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <math.h>

class Pioneer
{
  public:
    Pioneer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    virtual ~Pioneer();

  public:
    void odomPoseCallBack(const nav_msgs::Odometry& msg);
    void viconPoseCallBack(const geometry_msgs::TransformStamped& msg);
    void cmdVelHandPointCallBack(const geometry_msgs::Vector3& msg);
    void trajectoryCallBack(const trajectory_msgs::JointTrajectoryPoint& msg);

  protected:
    // parameters
    std::string HOSTNAME;
    std::string POSE;
    double HANDPOINT_OFFSET;  // metric: 10in = 0.25m
    double TUNE_K; // controller parameter

    // topic init
    ros::Subscriber odom_pose_sub;
    ros::Subscriber vicon_pose_sub;
    ros::Subscriber cmd_vel_hp_sub;
    ros::Subscriber traj_sub;

    ros::Publisher cmd_vel_pub;
    ros::Publisher cmd_vel_hp_pub;

    // pose info, theta from -3.14 to 3.14, left side is positive
    geometry_msgs::Pose2D pose_hp; // handpoint

    // velocity info
    geometry_msgs::Twist vel;  // v, w  (center) 
    geometry_msgs::Vector3 vel_hp;   // x, y  (hand point)
    geometry_msgs::Vector3 vel_hp_traj;  // x, y 
};

Pioneer::Pioneer(ros::NodeHandle& nh, ros::NodeHandle& nh_private):
  HOSTNAME(""), 
  POSE(""),
  HANDPOINT_OFFSET(0), 
  TUNE_K(0)
{
  nh_private.param( "machine", HOSTNAME, std::string("robot1") );
  ROS_INFO( "[%s]: pioneer launched.", HOSTNAME.c_str() );

  nh_private.param( "pose", POSE, std::string("odom") );
  if(POSE != "odom")
    ROS_INFO_STREAM( "[" << HOSTNAME << "]: set pose = " << POSE << " (localization by " << POSE << ")");

  nh_private.param( "handpoint_offset", HANDPOINT_OFFSET, 0.25);
  if(HANDPOINT_OFFSET != 0.25)
    ROS_INFO_STREAM( "[" << HOSTNAME << "]: set handpoint offset = " << HANDPOINT_OFFSET);

  nh_private.param( "tune", TUNE_K, 0.3);
  if(TUNE_K != 0.3)
    ROS_INFO_STREAM( "[" << HOSTNAME << "]: set tune K = " << TUNE_K);

  if(POSE == "vicon")
    vicon_pose_sub = nh.subscribe("/vicon/" + HOSTNAME + "/" + HOSTNAME, 1, &Pioneer::viconPoseCallBack, this);
  else 
    odom_pose_sub = nh.subscribe("RosAria/pose", 1, &Pioneer::odomPoseCallBack, this);
  cmd_vel_hp_sub = nh.subscribe("cmd_vel_hp", 1, &Pioneer::cmdVelHandPointCallBack, this);
  traj_sub = nh.subscribe("trajectory", 1, &Pioneer::trajectoryCallBack, this);

  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  cmd_vel_hp_pub = nh.advertise<geometry_msgs::Vector3>("cmd_vel_hp", 1);
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
  this->pose_hp.x = msg.transform.translation.x;;
  this->pose_hp.y = msg.transform.translation.y;;
  ROS_DEBUG_STREAM("vicon: theta="<<pose_hp.theta<<"; x_hp="<<pose_hp.x<<"; y_hp="<<pose_hp.y<<";\n");
}

void Pioneer::cmdVelHandPointCallBack(const geometry_msgs::Vector3& msg)
{
  this->vel_hp = msg;
  double x, y, v, w, theta;
  x = vel_hp.x; // linear velocity x, y in ground frame
  y = vel_hp.y;
  theta = this->pose_hp.theta;

  /** matrix transform
   *  v   1    L*cos0  L*sin0     x
   *    = - *                  * 
   *  w   L    -sin0    cos0      y
   */
  v = x*cos(theta) + y*sin(theta);
  w = ( x*(-sin(theta)) + y*cos(theta) )/HANDPOINT_OFFSET; 
  ROS_DEBUG_STREAM("theta="<<theta<<"; x="<<x<<"; y="<<y<<"; v="<<v<<"; w="<<w<<";\n");

  // pub vel
  vel.linear.x = v;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = w;
  cmd_vel_pub.publish(this->vel);
}

void Pioneer::trajectoryCallBack(const trajectory_msgs::JointTrajectoryPoint& msg)
{
  // get reference trajectory
  double x_ref, y_ref, vx_ref, vy_ref;
  x_ref = msg.positions[0];
  y_ref = msg.positions[1];
  vx_ref = msg.velocities[0];
  vy_ref = msg.velocities[1];

  // get the pose of hand point
  double x_hp, y_hp; 
  x_hp = this->pose_hp.x;
  y_hp = this->pose_hp.y;

  // tracking controller
  double ux, uy;
  ux = vx_ref - TUNE_K * (x_hp - x_ref);
  uy = vy_ref - TUNE_K * (y_hp - y_ref);
  ROS_DEBUG_STREAM("ux="<<ux<<"; uy="<<uy<<";\n");

  // pub vel_hp
  this->vel_hp_traj.x = ux;
  this->vel_hp_traj.y = uy;
  this->vel_hp_traj.z = 0;
  cmd_vel_hp_pub.publish(this->vel_hp_traj);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  Pioneer* pioneer = new Pioneer(nh, nh_private);
  ros::spin();

  delete pioneer;
  pioneer = NULL;

  return 0;
}
