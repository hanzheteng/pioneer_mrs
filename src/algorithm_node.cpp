#include <pioneer_mrs/pioneer.h>
#include <pioneer_mrs/CommunicationState.h>
#include <geometry_msgs/Vector3.h>

class Algorithm : public Pioneer
{
  public:
    Algorithm(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  public:
    void computeVelocity(double);
    void communicationStateCallBack(const pioneer_mrs::CommunicationState &);
    void robot1ViconPoseCallBack(const geometry_msgs::TransformStamped &);
    void robot2ViconPoseCallBack(const geometry_msgs::TransformStamped &);
    void robot3ViconPoseCallBack(const geometry_msgs::TransformStamped &);
    void robot4ViconPoseCallBack(const geometry_msgs::TransformStamped &);
    void robot5ViconPoseCallBack(const geometry_msgs::TransformStamped &);
  
  protected:
    void updatePose(int, geometry_msgs::TransformStamped);
    geometry_msgs::Vector3 sign(geometry_msgs::Pose2D, geometry_msgs::Pose2D);
    geometry_msgs::Vector3 gradient(geometry_msgs::Pose2D);
    
  protected:
    double Q[5];
    bool comm_state[5];
    geometry_msgs::Pose2D pose[5];
    geometry_msgs::Pose2D pose_offset;
    geometry_msgs::Vector3 vel_hp;

    ros::Subscriber comm_state_sub;
    ros::Subscriber vicon_sub[5];
    ros::Publisher vel_hp_pub;
};


Algorithm::Algorithm(ros::NodeHandle& nh, ros::NodeHandle& nh_private):
  Pioneer(nh, nh_private)
{
  for(int i=0; i<=4; i++)
    Q[i] = 0;

  comm_state_sub = nh.subscribe("comm_state", 1, &Algorithm::communicationStateCallBack, this);

  vicon_sub[0] = nh.subscribe("/vicon/robot1/robot1", 1, &Algorithm::robot1ViconPoseCallBack, this);
  vicon_sub[1] = nh.subscribe("/vicon/robot2/robot2", 1, &Algorithm::robot2ViconPoseCallBack, this);
  vicon_sub[2] = nh.subscribe("/vicon/robot3/robot3", 1, &Algorithm::robot3ViconPoseCallBack, this);
  vicon_sub[3] = nh.subscribe("/vicon/robot4/robot4", 1, &Algorithm::robot4ViconPoseCallBack, this);
  vicon_sub[4] = nh.subscribe("/vicon/robot5/robot5", 1, &Algorithm::robot5ViconPoseCallBack, this);

  vel_hp_pub = nh.advertise<geometry_msgs::Vector3>("cmd_vel_hp", 1);
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
  this->pose[index].theta = 0;
  this->pose[index].x = msg.transform.translation.x;
  this->pose[index].y = msg.transform.translation.y;
  ROS_DEBUG_STREAM("pose_r"<<index<<": x="<<pose[index].x<<"; y="<<pose[index].y<<";\n");
}

geometry_msgs::Vector3 Algorithm::sign(geometry_msgs::Pose2D m, geometry_msgs::Pose2D n)
{
  geometry_msgs::Vector3 vector;
  if(m.x > n.x)
    vector.x = 1;
  else if(m.x == n.x)
    vector.x = 0;
  else
    vector.x = -1;

  if(m.y > n.y)
    vector.y = 1;
  else if(m.y == n.y)
    vector.y = 0;
  else
    vector.y = -1;

  vector.z = 0;
  return vector;
}


geometry_msgs::Vector3 Algorithm::gradient(geometry_msgs::Pose2D input)
{
  double x = input.x, y = input.y;
  geometry_msgs::Vector3 vector;
  switch(HOSTNUM)
  {
    // f(x) = 0.5x^2 + 0.5y^2
    // g(x) = [x, y]
    case 0:
      vector.x = x;
      vector.y = y;
    
    // f(x) = 0.5(x-1)^2 + 0.5y^2
    // g(x) = [x-1, y]
    case 1:
      vector.x = x - 1;
      vector.y = y;

    // f(x) = 0.5(x+1)^2 + 0.5y^2
    // g(x) = [x+1, y]
    case 2:
      vector.x = x + 1;
      vector.y = y;

    // f(x) = 0.5x^2 + 0.5(y-1)^2
    // g(x) = [x, y-1]
    case 3:
      vector.x = x;
      vector.y = y - 1;

    // f(x) = 0.5x^2 + 0.5(y+1)^2
    // g(x) = [x, y+1]
    case 4:
      vector.x = x;
      vector.y = y + 1;

    default:
      vector.x = 0;
      vector.y = 0;
  }
  vector.z = 0;
  return vector;
}


void Algorithm::computeVelocity(double T)
{
  this->vel_hp.x = 0;
  this->vel_hp.y = 0;
  this->vel_hp.z = 0;
  geometry_msgs::Vector3 grad;
  geometry_msgs::Vector3 sgn;
  for(int i=0; i<=4; i++)
  {
    if(comm_state[i])
    {
      sgn = sign(pose[i], pose_hp);
      vel_hp.x += Q[i]*sgn.x;
      vel_hp.y += Q[i]*sgn.y;
      this->Q[i] += T;
    }
  }
  grad = gradient(pose_hp);
  vel_hp.x -= grad.x;
  vel_hp.y -= grad.y;
  vel_hp_pub.publish(this->vel_hp);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "algorithm_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  Algorithm* algorithm = new Algorithm(nh, nh_private);

  ros::Rate rate(20);
  while(ros::ok())
  {
    ros::spinOnce();
    algorithm->computeVelocity(1/20);
    rate.sleep();
  }

  delete algorithm;
  algorithm = NULL;

  return 0;
}
