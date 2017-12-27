#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

class Pioneer
{
  private:
    ros::Subscriber sub_vel_hp;
    ros::Subscriber sub_pose;
    ros::Publisher pub_vel;
    ros::Publisher pub_vel_hp;

    //velocity info
    geometry_msgs::Twist vel;  // v, w  (center) 
    geometry_msgs::Vector3 vel_hp;   // x, y  (hand point)

    //pose info, theta from -3.14 to 3.14, left side is positive
    geometry_msgs::Pose2D pose; // center
    geometry_msgs::Pose2D pose_hp; // handpoint
    geometry_msgs::Pose2D pose_cmd; // handpoint

    const float HANDPOINT_OFFSET = 0.25;  //by meter: 10in = 0.25m

  public:

    Pioneer(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    {
        sub_vel_hp = nh_private.subscribe("cmd_vel_hp", 1, &Pioneer::handPointCallBack, this);
        sub_pose = nh.subscribe("RosAria/pose", 1, &Pioneer::poseCallBack, this);

        pub_vel = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
        pub_vel_hp = nh_private.advertise<geometry_msgs::Vector3>("cmd_vel_hp", 1);
    }

    ~Pioneer()
    {

    }

    void poseCallBack(const nav_msgs::Odometry& msg) // from ROSARIA lib
    {
      //transform quaternion to Euler angle
      tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      this->pose.theta = yaw;  //yaw from -3.14 to 3.14, left side is the positive number
      this->pose.x = msg.pose.pose.position.x;
      this->pose.y = msg.pose.pose.position.y;
      ROS_INFO_STREAM("theta="<<pose.theta<<"; x="<<pose.x<<"; y="<<pose.y<<";\n");

      this->pose_hp.theta = this->pose.theta;
      this->pose_hp.x = this->pose.x + HANDPOINT_OFFSET * cos(pose.theta);
      this->pose_hp.y = this->pose.y + HANDPOINT_OFFSET * sin(pose.theta);
      ROS_INFO_STREAM("theta="<<pose_hp.theta<<"; x_hp="<<pose_hp.x<<"; y_hp="<<pose_hp.y<<";\n");
    }

    void handPointCallBack(const geometry_msgs::Vector3& msg) // pub to RosAria/cmd_vel
    {
      this->vel_hp = msg;

      float x, y, v, w, theta;
      x = vel_hp.x; // linear velocity x, y in ground frame
      y = vel_hp.y;
      theta = this->pose.theta;
/*
 v   1    L*cos0  L*sin0     x
   = - *                  * 
 w   L    -sin0    cos0      y
*/  
      v = x*cos(theta) + y*sin(theta);
      w = ( x*(-sin(theta)) + y*cos(theta) )/HANDPOINT_OFFSET; 

      ROS_DEBUG_STREAM("theta="<<theta<<"; x="<<x<<"; y="<<y<<"; v="<<v<<"; w="<<w<<";\n");
      vel.linear.x = v;
      vel.linear.y = 0;
      vel.linear.z = 0;
      vel.angular.x = 0;
      vel.angular.y = 0;
      vel.angular.z = w;
      pub_vel.publish(this->vel);
    }

    geometry_msgs::Pose2D getPoseHp()
    {
      return this->pose_hp;
    }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Pioneer* pioneer = new Pioneer(nh, nh_private);
  ros::spin();
}
