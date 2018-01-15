#include <ros/ros.h> 
#include <pioneer_mrs/CommunicationState.h>

class Communication
{
  public:
    Communication(ros::NodeHandle& nh);

    pioneer_mrs::CommunicationState comm_state[5];
    ros::Publisher comm_pub[5];
};


Communication::Communication(ros::NodeHandle& nh)
{
  comm_state[0].state = {false, true, false, false, true};
  comm_state[1].state = {true, false, true, false, false};
  comm_state[2].state = {false, true, false, true, false};
  comm_state[3].state = {false, false, true, false, true};
  comm_state[4].state = {true, false, false, true, false};
  comm_pub[0] = nh.advertise<pioneer_mrs::CommunicationState>("/robot1/comm_state", 1);
  comm_pub[1] = nh.advertise<pioneer_mrs::CommunicationState>("/robot2/comm_state", 1);
  comm_pub[2] = nh.advertise<pioneer_mrs::CommunicationState>("/robot3/comm_state", 1);
  comm_pub[3] = nh.advertise<pioneer_mrs::CommunicationState>("/robot4/comm_state", 1);
  comm_pub[4] = nh.advertise<pioneer_mrs::CommunicationState>("/robot5/comm_state", 1);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "comm_state_cmd");
  ros::NodeHandle nh;
  Communication communication(nh);
  ros::Rate rate(1);
  while(ros::ok())
  {
    for(int i=0; i<=4; i++)
      communication.comm_pub[i].publish(communication.comm_state[i]);
    rate.sleep();
  }
  return 0;
}
