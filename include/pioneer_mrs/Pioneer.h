#ifndef PIONEER_MRS_INCLUDE_PIONEER_MRS_PIONEER_H_
#define PIONEER_MRS_INCLUDE_PIONEER_MRS_PIONEER_H_

#include <ros/ros.h> 
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <math.h>

class Pioneer
{
  private:
    ros::Subscriber sub_pose;
    ros::Subscriber sub_vel_hp;
    ros::Subscriber sub_traj;
    ros::Publisher pub_vel;
    ros::Publisher pub_vel_hp_traj;

    //velocity info
    geometry_msgs::Twist vel;  // v, w  (center) 
    geometry_msgs::Vector3 vel_hp;   // x, y  (hand point)
    geometry_msgs::Vector3 vel_hp_traj;  // x, y 

    //pose info, theta from -3.14 to 3.14, left side is positive
    geometry_msgs::Pose2D pose; // center
    geometry_msgs::Pose2D pose_hp; // handpoint

    const float HANDPOINT_OFFSET = 0.25;  // metric: 10in = 0.25m
    const float TUNE_K = 0.3; // controller parameter

  public:

    Pioneer(ros::NodeHandle & nh)
    {
        sub_pose = nh.subscribe("RosAria/pose", 1, &Pioneer::poseCallBack, this);
        sub_vel_hp = nh.subscribe("cmd_vel_hp", 1, &Pioneer::handPointCallBack, this);
        sub_traj = nh.subscribe("trajectory", 1, &Pioneer::trajectoryCallBack, this);

        pub_vel = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
        pub_vel_hp_traj = nh.advertise<geometry_msgs::Vector3>("cmd_vel_hp", 1);
    }
    ~Pioneer()
    {
    }

    void poseCallBack(const nav_msgs::Odometry& msg)
    {
      //transform quaternion to Euler angle
      tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      this->pose.theta = yaw;  //yaw from -3.14 to 3.14, left side is the positive number
      this->pose.x = msg.pose.pose.position.x;
      this->pose.y = msg.pose.pose.position.y;
      ROS_DEBUG_STREAM("theta="<<pose.theta<<"; x="<<pose.x<<"; y="<<pose.y<<";\n");

      this->pose_hp.theta = this->pose.theta;
      this->pose_hp.x = this->pose.x + HANDPOINT_OFFSET * cos(pose.theta);
      this->pose_hp.y = this->pose.y + HANDPOINT_OFFSET * sin(pose.theta);
      ROS_INFO_STREAM("theta="<<pose_hp.theta<<"; x_hp="<<pose_hp.x<<"; y_hp="<<pose_hp.y<<";\n");
    }

    void handPointCallBack(const geometry_msgs::Vector3& msg)
    {
      this->vel_hp = msg;
      float x, y, v, w, theta;
      x = vel_hp.x; // linear velocity x, y in ground frame
      y = vel_hp.y;
      theta = this->pose.theta;

      // matrix transform
      //  v   1    L*cos0  L*sin0     x
      //    = - *                  * 
      //  w   L    -sin0    cos0      y
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
      pub_vel.publish(this->vel);
    }

    void trajectoryCallBack(const trajectory_msgs::JointTrajectoryPoint& msg)
    {
      // get reference trajectory
      float x_ref, y_ref, vx_ref, vy_ref;
      x_ref = msg.positions[0];
      y_ref = msg.positions[1];
      vx_ref = msg.velocities[0];
      vy_ref = msg.velocities[1];

      // get the pose of hand point
      float x_hp, y_hp; 
      x_hp = this->pose_hp.x;
      y_hp = this->pose_hp.y;

      // tracking controller
      float ux, uy;
      ux = vx_ref - TUNE_K * (x_hp - x_ref);
      uy = vy_ref - TUNE_K * (y_hp - y_ref);
      ROS_INFO_STREAM("ux="<<ux<<"; uy="<<uy<<";\n");

      // pub vel_hp
      this->vel_hp_traj.x = ux;
      this->vel_hp_traj.y = uy;
      this->vel_hp_traj.z = 0;
      pub_vel_hp_traj.publish(this->vel_hp_traj);
    }

};


#endif /* PIONEER_MRS_INCLUDE_PIONEER_MRS_PIONEER_H_ */
