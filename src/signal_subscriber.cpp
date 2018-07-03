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
  ros::Publisher  mps_pub = n.advertise<mps_driver::MPS>("/mps_data", 1);
  ros::Publisher  GPS_pub = n.advertise<sensor_msgs::NavSatFix>("mavros/global_position/global", 1);
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

    mps_driver::MPS mps;
    mps.gasID = "Methane";
    mps.percentLEL = 20.12345678;
    mps.pressure = 10.2;
    mps.temperature = 24.65;
    mps.GPS_latitude = 40.1331311;
    mps.GPS_longitude = -111.4512464;
    mps.GPS_altitude = 2.1310131;
    mps_pub.publish(mps);

    sensor_msgs::NavSatFix gps;
    gps.status.status = 0;
    gps.latitude = 33.333333;
    gps.longitude = 44.444444;
    gps.altitude = 22.222222;
    GPS_pub.publish(gps);
    
    enif_iuc::AgentTakeoff command;
    command.agent_number = count%3+1;
    command.takeoff_command.data = true;
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
