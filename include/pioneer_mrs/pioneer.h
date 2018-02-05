#ifndef PIONEER_MRS_PIONEER_H_
#define PIONEER_MRS_PIONEER_H_

#include <ros/ros.h> 
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>

#include <math.h>
#include <tf/transform_broadcaster.h>


class Pioneer
{
  public:
    Pioneer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    virtual ~Pioneer();

  public:
    void odomPoseCallBack(const nav_msgs::Odometry &);
    void viconPoseCallBack(const geometry_msgs::TransformStamped &);

  protected:
    // parameters
    int HOSTNUM;  // array index
    std::string HOSTNAME;
    std::string POSE;
    double HANDPOINT_OFFSET;  // metric: 10in = 0.25m

    // pose info, theta from -3.14 to 3.14, left side is positive
    geometry_msgs::Pose2D pose_hp; // handpoint pose

    // topic init
    ros::Subscriber odom_pose_sub;
    ros::Subscriber vicon_pose_sub;
};


#endif /* PIONEER_MRS_PIONEER_H_ */