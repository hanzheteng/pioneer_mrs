#include <pioneer_mrs/pioneer.h>
#include <pioneer_mrs/CommunicationState.h>
#include <pioneer_mrs/MissionState.h>
#include <pioneer_mrs/Point2D.h>
#include <geometry_msgs/Vector3.h>

class Algorithm : public Pioneer
{
  public:
    Algorithm(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  public:
    void computeVelocity(double);
    void missionStateCallBack(const pioneer_mrs::MissionState &);
    void communicationStateCallBack(const pioneer_mrs::CommunicationState &);
    void robot1ViconPoseCallBack(const geometry_msgs::TransformStamped &);
    void robot2ViconPoseCallBack(const geometry_msgs::TransformStamped &);
    void robot3ViconPoseCallBack(const geometry_msgs::TransformStamped &);
    void robot4ViconPoseCallBack(const geometry_msgs::TransformStamped &);
    void robot5ViconPoseCallBack(const geometry_msgs::TransformStamped &);
  
  protected:
    void updatePose(int, geometry_msgs::TransformStamped);
    Point2D sign(Point2D);
    Point2D gradient(Point2D);
    
  protected:
    double Q[5];
    bool STATE;
    bool comm_state[5];
    Point2D pose_i;
    Point2D pose_j[5];
    Point2D pose_offset[5];
    geometry_msgs::Vector3 vel_hp;

    ros::Subscriber miss_state_sub;
    ros::Subscriber comm_state_sub;
    ros::Subscriber vicon_sub[5];
    ros::Publisher vel_hp_pub;
};


Algorithm::Algorithm(ros::NodeHandle& nh, ros::NodeHandle& nh_private):
  Pioneer(nh, nh_private),
  STATE(false),
  pose_i(pose_hp.x, pose_hp.y)
{
  pose_offset[0].x = 0;
  pose_offset[0].y = 0;

  pose_offset[1].x = 1;
  pose_offset[1].y = 0;

  pose_offset[2].x = -1;
  pose_offset[2].y = 0;

  pose_offset[3].x = 0;
  pose_offset[3].y = 1;

  pose_offset[4].x = 0;
  pose_offset[4].y = -1;

  pose_i = pose_i + pose_offset[HOSTNUM];

  for(int i=0; i<=4; i++)
    Q[i] = 0;

  miss_state_sub = nh.subscribe("mission_state", 1, &Algorithm::missionStateCallBack, this);
  comm_state_sub = nh.subscribe("comm_state", 1, &Algorithm::communicationStateCallBack, this);

  vicon_sub[0] = nh.subscribe("/vicon/robot1/robot1", 1, &Algorithm::robot1ViconPoseCallBack, this);
  vicon_sub[1] = nh.subscribe("/vicon/robot2/robot2", 1, &Algorithm::robot2ViconPoseCallBack, this);
  vicon_sub[2] = nh.subscribe("/vicon/robot3/robot3", 1, &Algorithm::robot3ViconPoseCallBack, this);
  vicon_sub[3] = nh.subscribe("/vicon/robot4/robot4", 1, &Algorithm::robot4ViconPoseCallBack, this);
  vicon_sub[4] = nh.subscribe("/vicon/robot5/robot5", 1, &Algorithm::robot5ViconPoseCallBack, this);

  vel_hp_pub = nh.advertise<geometry_msgs::Vector3>("cmd_vel_hp", 1);
}

void Algorithm::missionStateCallBack(const pioneer_mrs::MissionState& msg)
{
  this->STATE = msg.algorithm;
}


void Algorithm::communicationStateCallBack(const pioneer_mrs::CommunicationState& msg)
{
  for(int i=0; i<=4; i++)
    this->comm_state[i] = msg.state[i];
}


void Algorithm::robot1ViconPoseCallBack(const geometry_msgs::TransformStamped& msg)
{
  updatePose(0,msg);
}
void Algorithm::robot2ViconPoseCallBack(const geometry_msgs::TransformStamped& msg)
{
  updatePose(1,msg);
}
void Algorithm::robot3ViconPoseCallBack(const geometry_msgs::TransformStamped& msg)
{
  updatePose(2,msg);
}
void Algorithm::robot4ViconPoseCallBack(const geometry_msgs::TransformStamped& msg)
{
  updatePose(3,msg);
}
void Algorithm::robot5ViconPoseCallBack(const geometry_msgs::TransformStamped& msg)
{
  updatePose(4,msg);
}


void Algorithm::updatePose(int index, geometry_msgs::TransformStamped msg)
{
  this->pose_j[index].x = msg.transform.translation.x;
  this->pose_j[index].y = msg.transform.translation.y;
  ROS_DEBUG_STREAM("pose_r"<<index<<": x="<<pose_j[index].x<<"; y="<<pose_j[index].y<<";\n");
}

Point2D Algorithm::sign(Point2D input)
{
  Point2D point;
  if(input.x > 0)
    point.x = 1;
  else if(input.x == 0)
    point.x = 0;
  else
    point.x = -1;

  if(input.y > 0)
    point.y = 1;
  else if(input.y == 0)
    point.y = 0;
  else
    point.y = -1;

  return point;
}


Point2D Algorithm::gradient(Point2D input)
{
  double x = input.x, y = input.y;
  Point2D point;
  switch(HOSTNUM)
  {
    // f(x) = 0.5x^2 + 0.5y^2
    // g(x) = [x, y]
    case 0:
      point.x = x;
      point.y = y;
    
    // f(x) = 0.5(x-1)^2 + 0.5y^2
    // g(x) = [x-1, y]
    case 1:
      point.x = x - 1;
      point.y = y;

    // f(x) = 0.5(x+1)^2 + 0.5y^2
    // g(x) = [x+1, y]
    case 2:
      point.x = x + 1;
      point.y = y;

    // f(x) = 0.5x^2 + 0.5(y-1)^2
    // g(x) = [x, y-1]
    case 3:
      point.x = x;
      point.y = y - 1;

    // f(x) = 0.5x^2 + 0.5(y+1)^2
    // g(x) = [x, y+1]
    case 4:
      point.x = x;
      point.y = y + 1;
  }
  return point;
}


void Algorithm::computeVelocity(double T)
{
  if(STATE)
  {
    Point2D vel, sgn, grad;
    for(int i=0; i<=4; i++)
    {
      if(comm_state[i])
      {
        sgn = sign( (pose_j[i] + pose_offset[i]) - pose_i) * Q[i];
        vel = vel + sgn;
        this->Q[i] += T;
      }
    }
    grad = gradient(pose_i);
    vel = vel - grad;
    ROS_DEBUG_STREAM("Vel2D: x="<<vel.x<<"; y="<<vel.y<<";\n");

    vel_hp.x = vel.x;
    vel_hp.y = vel.y;
    vel_hp.z = 0;
    vel_hp_pub.publish(this->vel_hp);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "algorithm_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Algorithm* algorithm = new Algorithm(nh, nh_private);

  double T = 0.05; // sample period
  ros::Rate rate(1/T);
  while(ros::ok())
  {
    ros::spinOnce();
    algorithm->computeVelocity(T);
    rate.sleep();
  }

  delete algorithm;
  algorithm = NULL;

  return 0;
}
