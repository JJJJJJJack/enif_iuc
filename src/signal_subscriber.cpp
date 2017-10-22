#include "enif.h"

#include "enif_iuc/AgentTakeoff.h"
#include "enif_iuc/AgentWaypointTask.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "signal_subscriber");
  ros::NodeHandle n;
	
  ros::Rate loop_rate(1000);
  ros::Publisher  stuff_pub2 = n.advertise<std_msgs::UInt8>("/gro", 1000);
  ros::Publisher  stuff_pub = n.advertise<enif_iuc::AgentTakeoff>("/takeoff_command", 1);
  ros::Publisher  wp_pub = n.advertise<enif_iuc::AgentWaypointTask>("/waypoint_list", 1);
  int count = 0;
  
  while (ros::ok())
  {
    enif_iuc::AgentWaypointTask wpmsg;
    wpmsg.agent_number = count%3+1;
    wpmsg.waypoint_list.velocity = 2.0;
    wpmsg.waypoint_list.damping_distance = 1.5;
    enif_iuc::Waypoint wp;
    wp.latitude = 40.35353535;
    wp.longitude = -111.54545454;
    wp.target_height = 2.2;
    wp.heading = 60;
    wp.staytime = 5;
    wpmsg.waypoint_list.mission_waypoint.push_back(wp);
    wp_pub.publish(wpmsg);
    
    enif_iuc::AgentTakeoff command;
    command.agent_number = count%3+1;
    command.takeoff_command = true;
    //stuff_pub.publish(command);
    std_msgs::UInt8 data;
    data.data = 9;
    //stuff_pub2.publish(data);
    //cout<<"hi"<<endl;
    ros::spinOnce();
    
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
