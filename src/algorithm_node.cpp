#include <pioneer_mrs/pioneer.h>
#include <pioneer_mrs/CommunicationState.h>
#include <pioneer_mrs/MissionState.h>
#include <pioneer_mrs/Point2D.h>
#include <pioneer_mrs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <dynamic_reconfigure/server.h>
#include <pioneer_mrs/FormationConfig.h>

class Algorithm : public Pioneer
{
  public:
    Algorithm(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~Algorithm();

  public:
    void computeVelocity(double);
    void missionStateCallBack(const pioneer_mrs::MissionState &);
    void communicationStateCallBack(const pioneer_mrs::CommunicationState &);
    void dynamicReconfigureCallBack(pioneer_mrs::FormationConfig &, uint32_t);
    void updateNeighborPose();
  
  protected:
    Point2D sign(Point2D);
    Point2D gradient(Point2D);
    
  protected:
    double Q;
    double FORMATION_OFFSET;
    Point2D pose_i;
    Point2D pose_offset[5];

    bool STATE;
    ros::Subscriber miss_state_sub;

    bool comm_state[5];
    ros::Subscriber comm_state_sub;

    Point2D pose_j[5];
    ros::ServiceClient get_pose[5];

    geometry_msgs::Vector3 vel_hp;
    ros::Publisher vel_hp_pub;

    dynamic_reconfigure::Server<pioneer_mrs::FormationConfig> *dynamic_reconfigure_server;
};


Algorithm::Algorithm(ros::NodeHandle& nh, ros::NodeHandle& nh_private):
  Pioneer(nh, nh_private),
  Q(0.2),
  STATE(false),
  FORMATION_OFFSET(0)
{
  pose_offset[0].x = -1;
  pose_offset[0].y = 1;

  pose_offset[1].x = 1;
  pose_offset[1].y = 1;

  pose_offset[2].x = 0;
  pose_offset[2].y = 0;

  pose_offset[3].x = 1;
  pose_offset[3].y = -1;

  pose_offset[4].x = -1;
  pose_offset[4].y = -1;

  miss_state_sub = nh.subscribe("mission_state", 1, &Algorithm::missionStateCallBack, this);
  comm_state_sub = nh.subscribe("comm_state", 1, &Algorithm::communicationStateCallBack, this);

  for(int i=0;i<=4;i++)
  {
    if(i != HOSTNUM)
      get_pose[i] = nh.serviceClient<pioneer_mrs::Pose2D>("/robot" + std::to_string(i+1) + "/get_pose", true); 
  }   // second argument is true ==> set persistent connections to TCP
  
  vel_hp_pub = nh.advertise<geometry_msgs::Vector3>("cmd_vel_hp", 1);

  dynamic_reconfigure_server = new dynamic_reconfigure::Server<pioneer_mrs::FormationConfig>;
  dynamic_reconfigure_server->setCallback(boost::bind(&Algorithm::dynamicReconfigureCallBack, this, _1, _2));
}

Algorithm::~Algorithm()
{
  delete dynamic_reconfigure_server;
  dynamic_reconfigure_server = NULL;
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

void Algorithm::dynamicReconfigureCallBack(pioneer_mrs::FormationConfig &config, uint32_t level)
{
  double value;
  value = config.formation_offset;
  if(value != this->FORMATION_OFFSET && value > 0)
  {
    ROS_INFO_STREAM(HOSTNAME + " Setting FORMATION_OFFSET: "<<value);
    this->FORMATION_OFFSET = value;
  }
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
    // f(x) = 0.5(x+1)^2 + 0.5(y-1)^2
    // g(x) = [x+1, y-1]
    case 0:
      point.x = x + 1 - FORMATION_OFFSET;
      point.y = y - 1;
      break;
    
    // f(x) = 0.5(x-1)^2 + 0.5(y-1)^2
    // g(x) = [x-1, y-1]
    case 1:
      point.x = x - 1 - FORMATION_OFFSET;
      point.y = y - 1;
      break;

    // f(x) = 0.5x^2 + 0.5y^2
    // g(x) = [x, y]
    case 2:
      point.x = x - FORMATION_OFFSET;
      point.y = y;
      break;

    // f(x) = 0.5(x-1)^2 + 0.5(y+1)^2
    // g(x) = [x-1, y+1]
    case 3:
      point.x = x - 1 - FORMATION_OFFSET;
      point.y = y + 1;
      break;

    // f(x) = 0.5(x+1)^2 + 0.5(y+1)^2
    // g(x) = [x+1, y+1]
    case 4:
      point.x = x + 1 - FORMATION_OFFSET;
      point.y = y + 1;
      break;

    default:
      point.x = x;
      point.y = y;
      ROS_ERROR_STREAM(HOSTNAME + " algorithm_node: switch function ERROR. ");
      break;
  }
  return point;
}


void Algorithm::updateNeighborPose()
{
  // update neighbor pose
  ROS_DEBUG_STREAM(HOSTNAME + " algorithm_node: Updating neighbor pose.");
  pioneer_mrs::Pose2D srv[5];
  for(int i=0;i<=4;i++)
  {
    if(i != HOSTNUM)
      get_pose[i].call(srv[i]);
  }
  for(int i=0;i<=4;i++)
  {
    if(srv[i].response.success)
    {
      pose_j[i].x = srv[i].response.x;
      pose_j[i].y = srv[i].response.y;
    }
  }
  ROS_DEBUG_STREAM(HOSTNAME + " algorithm_node: Updated neighbor pose.");
  
  // update my pose
  pose_i.x = pose_hp.x;
  pose_i.y = pose_hp.y;
}


void Algorithm::computeVelocity(double T)
{
  if(STATE)
  {
    updateNeighborPose();
    pose_i = pose_i - pose_offset[HOSTNUM];
    Point2D vel, sgn, grad;
    // sgn makes the team reach a consensus
    // grad makes the consensus point stick to the optimal (desired) point
    for(int i=0; i<=4; i++)
    {
      if(comm_state[i])
      {
        //sgn = sign( (pose_j[i] - pose_offset[i]) - pose_i)* Q;
        sgn = ( (pose_j[i] - pose_offset[i]) - pose_i ) * Q;
        vel = vel + sgn;
        ROS_DEBUG_STREAM(HOSTNAME + " algorithm_node: sgn"<<i+1<<"  Vx="<<sgn.x<<"; Vy="<<sgn.y<<";");
        //this->Q += T;
      }
    }
    grad = gradient( pose_i );
    vel = vel - grad;
    ROS_DEBUG_STREAM(HOSTNAME + " algorithm_node: grad  Gx="<<grad.x<<"; Gy="<<grad.y<<";");
    ROS_DEBUG_STREAM(HOSTNAME + " algorithm_node: Vel2D Vx="<<vel.x<<"; Vy="<<vel.y<<";\n");

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

  double T = 0.1; // sample period
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
