#include <pioneer_mrs/pioneer.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Vector3.h>

class Trajectory : public Pioneer
{
  public:
    Trajectory(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  public:
    void trajectoryCallBack(const trajectory_msgs::JointTrajectoryPoint &);

  protected:
    // controller parameter
    double TUNE_K;

    // vel to pub
    geometry_msgs::Vector3 vel_hp;  // x, y 

    // topic init
    ros::Subscriber traj_sub;
    ros::Publisher vel_hp_pub;
};


Trajectory::Trajectory(ros::NodeHandle& nh, ros::NodeHandle& nh_private):
  Pioneer(nh, nh_private),
  TUNE_K(0)
{
  nh_private.param( "tune", TUNE_K, 0.3);
  if(TUNE_K != 0.3)
    ROS_INFO_STREAM( "[" << HOSTNAME << "]: set tune K = " << TUNE_K);
  
  traj_sub = nh.subscribe("trajectory", 1, &Trajectory::trajectoryCallBack, this);
  vel_hp_pub = nh.advertise<geometry_msgs::Vector3>("cmd_vel_hp", 1);
}


void Trajectory::trajectoryCallBack(const trajectory_msgs::JointTrajectoryPoint& msg)
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
  this->vel_hp.x = ux;
  this->vel_hp.y = uy;
  this->vel_hp.z = 0;
  vel_hp_pub.publish(this->vel_hp);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  Trajectory* trajectory = new Trajectory(nh, nh_private);
  ros::spin();

  delete trajectory;
  trajectory = NULL;

  return 0;
}
