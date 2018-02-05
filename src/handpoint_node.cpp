#include <pioneer_mrs/pioneer.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <pioneer_mrs/Pose2D.h>

class HandPoint : public Pioneer
{
  public:
    HandPoint(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  public:
    void cmdVelHandPointCallBack(const geometry_msgs::Vector3 &);
    bool poseServiceCallBack(pioneer_mrs::Pose2DRequest &, pioneer_mrs::Pose2DResponse &);

  protected:
    // velocity info
    geometry_msgs::Twist vel;  // v, w  (center) 
    geometry_msgs::Vector3 vel_hp;   // x, y  (hand point)

    // topic init
    ros::Subscriber vel_hp_sub;
    ros::Publisher vel_pub;
    ros::ServiceServer pose_server;
};


HandPoint::HandPoint(ros::NodeHandle& nh, ros::NodeHandle& nh_private):
  Pioneer(nh, nh_private)
{
  vel_hp_sub = nh.subscribe("cmd_vel_hp", 1, &HandPoint::cmdVelHandPointCallBack, this);
  vel_pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);

  pose_server = nh.advertiseService("get_pose", &HandPoint::poseServiceCallBack, this);
}


void HandPoint::cmdVelHandPointCallBack(const geometry_msgs::Vector3& msg)
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
  vel_pub.publish(this->vel);
}

bool HandPoint::poseServiceCallBack(pioneer_mrs::Pose2DRequest &rqt, pioneer_mrs::Pose2DResponse &res)
{
  res.theta = this->pose_hp.theta;
  res.x = this->pose_hp.x;
  res.y = this->pose_hp.y;
  res.success = true;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "handpoint_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  HandPoint* handpoint = new HandPoint(nh, nh_private);
  ros::spin();

  delete handpoint;
  handpoint = NULL;

  return 0;
}