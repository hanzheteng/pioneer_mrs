#include <ros/ros.h> 
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose2D.h> //msg
#include <pioneer_mrs/Pose2D.h>  //srv
#include <dynamic_reconfigure/server.h>
#include <pioneer_mrs/PioneerConfig.h>

class Pioneer
{
  public:
    Pioneer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    virtual ~Pioneer();

  /* role one: provide pose information */
  public:
    void odomPoseCallBack(const nav_msgs::Odometry &);
    void gazeboPoseCallBack(const nav_msgs::Odometry &);
    void viconPoseCallBack(const geometry_msgs::TransformStamped &);
    bool poseServiceCallBack(pioneer_mrs::Pose2DRequest &, pioneer_mrs::Pose2DResponse &);

  protected:
    // pose parameters
    int HOSTNUM;  // array index
    std::string HOSTNAME;
    std::string POSE;

    // pose info, theta from -3.14 to 3.14, left side is positive
    geometry_msgs::Pose2D pose_hp; // handpoint pose

    // pose subscriber
    ros::Subscriber odom_pose_sub;
    ros::Subscriber gazebo_pose_sub;
    ros::Subscriber vicon_pose_sub;
    // pose server
    ros::ServiceServer pose_server;

  /* role two: convert vel_hp to vel (of center point) */
  public:
    void handpointOffsetReconfigureCallBack(pioneer_mrs::PioneerConfig &, uint32_t);
    void handpointVelocityCallBack(const geometry_msgs::Vector3 &);

  protected:
    // handpoint parameter
    double HANDPOINT_OFFSET;  // metric: 10in = 0.25m
    // reconfigure server
    dynamic_reconfigure::Server<pioneer_mrs::PioneerConfig> *cfg_server;

    // subscribe handpoint velocity
    ros::Subscriber vel_hp_sub;
    geometry_msgs::Vector3 vel_hp;   // x, y  (hand point)
    // publish center velocity
    ros::Publisher vel_pub;
    geometry_msgs::Twist vel;  // v, w  (center)
};


Pioneer::Pioneer(ros::NodeHandle& nh, ros::NodeHandle& nh_private):
  HOSTNUM(0),
  HOSTNAME(""), 
  POSE(""),
  HANDPOINT_OFFSET(0)
{
  // HOSTNAME
  nh_private.param( "hostname", HOSTNAME, std::string("robot1") );

  // HOSTNUM
  HOSTNUM = std::stoi( HOSTNAME.substr(5) ) - 1; // array index

  // POSE mode
  nh_private.param( "pose", POSE, std::string("odom") );
  if(POSE != "odom")
    ROS_INFO_STREAM( HOSTNAME + " set pose = " << POSE << " (localization by " << POSE << ")");
  if(POSE == "vicon")
    vicon_pose_sub = nh.subscribe("/vicon/" + HOSTNAME + "/" + HOSTNAME, 1, &Pioneer::viconPoseCallBack, this);
  else if(POSE == "gazebo")
    gazebo_pose_sub = nh.subscribe("gazebo/odom", 1, &Pioneer::gazeboPoseCallBack, this);
  else
    odom_pose_sub = nh.subscribe("RosAria/pose", 1, &Pioneer::odomPoseCallBack, this);

  // handpoint offset
  nh_private.param( "handpoint_offset", HANDPOINT_OFFSET, 0.25);
  if(HANDPOINT_OFFSET != 0.25)
    ROS_INFO_STREAM( HOSTNAME + " set handpoint offset = " << HANDPOINT_OFFSET);

  // handpoint offset reconfigure server
  cfg_server = new dynamic_reconfigure::Server<pioneer_mrs::PioneerConfig>;
  cfg_server->setCallback(boost::bind(&Pioneer::handpointOffsetReconfigureCallBack, this, _1, _2));

  // handpoint velocity callback
  vel_hp_sub = nh.subscribe("cmd_vel_hp", 1, &Pioneer::handpointVelocityCallBack, this);
  vel_pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

  // pose server
  pose_server = nh.advertiseService("get_pose", &Pioneer::poseServiceCallBack, this);

  // done
  ROS_INFO_STREAM( HOSTNAME + " pioneer_server launched." );
}


Pioneer::~Pioneer()
{
  delete cfg_server;
  cfg_server = NULL;
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
  pose.y = pose.y - (HOSTNUM - 2); // initialization offset: put robot 1-5 on a row, with 1 meter space
  this->pose_hp.x = pose.x + HANDPOINT_OFFSET * cos(pose.theta);
  this->pose_hp.y = pose.y + HANDPOINT_OFFSET * sin(pose.theta);
  ROS_DEBUG_STREAM(HOSTNAME + " odom: theta="<<pose_hp.theta<<"; x_hp="<<pose_hp.x<<"; y_hp="<<pose_hp.y<<";\n");
}


void Pioneer::gazeboPoseCallBack(const nav_msgs::Odometry& msg)
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
  ROS_DEBUG_STREAM(HOSTNAME + " gazebo: theta="<<pose_hp.theta<<"; x_hp="<<pose_hp.x<<"; y_hp="<<pose_hp.y<<";\n");
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
  ROS_DEBUG_STREAM(HOSTNAME + " vicon: theta="<<pose_hp.theta<<"; x_hp="<<pose_hp.x<<"; y_hp="<<pose_hp.y<<";\n");
}


bool Pioneer::poseServiceCallBack(pioneer_mrs::Pose2DRequest &rqt, pioneer_mrs::Pose2DResponse &res)
{
  res.theta = this->pose_hp.theta;
  res.x = this->pose_hp.x;
  res.y = this->pose_hp.y;
  res.success = true;
  return true;
}


void Pioneer::handpointOffsetReconfigureCallBack(pioneer_mrs::PioneerConfig &config, uint32_t level)
{
  double value;
  value = config.handpoint_offset;
  if(value != this->HANDPOINT_OFFSET && value > 0)
  {
    ROS_INFO_STREAM(HOSTNAME + " Set HANDPOINT_OFFSET = "<<value<<" m");
    this->HANDPOINT_OFFSET = value;
  }
}


void Pioneer::handpointVelocityCallBack(const geometry_msgs::Vector3& msg)
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
  ROS_DEBUG_STREAM(HOSTNAME + " vel: theta="<<theta<<"; x="<<x<<"; y="<<y<<"; v="<<v<<"; w="<<w<<";\n");

  // pub vel
  vel.linear.x = v;
  vel.linear.y = 0;
  vel.linear.z = 0;
  vel.angular.x = 0;
  vel.angular.y = 0;
  vel.angular.z = w;
  vel_pub.publish(this->vel);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer_server");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  Pioneer* pioneer = new Pioneer(nh, nh_private);
  ros::spin();

  delete pioneer;
  pioneer = NULL;

  return 0;
}