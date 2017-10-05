#include "enif.h"

//#include "enif_iuc/AgentTakeoff.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "signal_subscriber");
  ros::NodeHandle n;
	
  ros::Rate loop_rate(1000);
  ros::Publisher  stuff_pub2 = n.advertise<std_msgs::UInt8>("/gro", 1000);

  int count = 0;
  
  while (ros::ok())
  {
    //ros::Publisher  stuff_pub = n.advertise<enif_iuc::AgentTakeoff>("/ground/takeoff_command", 1);
    
    //enif_iuc::AgentTakeoff command;
    //command.agent_number = count%3+1;
    //command.takeoff_command = true;
    //stuff_pub.publish(command);
    std_msgs::UInt8 data;
    data.data = 9;
    stuff_pub2.publish(data);
    //cout<<"hi"<<endl;
    ros::spinOnce();
    
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
