#include "enif.h"

#include "enif_iuc/AgentTakeoff.h"
#include "enif_iuc/AgentWaypointTask.h"
#include "enif_iuc/AgentGlobalPosition.h"
#include "enif_iuc/AgentMPS.h"
#include "enif_iuc/AgentState.h"
#include "enif_iuc/AgentHeight.h"
#include "enif_iuc/AgentBatteryState.h"


bool NEW_TAKEOFF = false, NEW_WP = false;
enif_iuc::AgentTakeoff agent_takeoff;
enif_iuc::AgentWaypointTask agent_wp;

using namespace std;

void takeoff_callback(const enif_iuc::AgentTakeoff &new_message)
{
  agent_takeoff = new_message;
  NEW_TAKEOFF = true;
}

void wp_callback(const enif_iuc::AgentWaypointTask &new_message)
{
  agent_wp = new_message;
  NEW_WP = true;
}

void get_state(char* buf)
{
  state.data = CharToInt(buf[3]);
}

void get_mps(char* buf)
{
  GAS_ID = CharToInt(buf[3]);
  if(GAS_ID == GAS_PROPANE){
    string str = "Propane";
    mps.gasID = str;
  }else if(GAS_ID == GAS_METHANE){
    string str = "Methane";
    mps.gasID = str;
  }else{
    string str = "None";
    mps.gasID = str;
  }
  float percentLEL, temperature, pressure, humidity, GPS_latitude, GPS_longitude;
  CharToFloat(buf+4, percentLEL);
  mps.percentLEL = percentLEL;
  CharToFloat(buf+4+4, temperature);
  mps.temperature = temperature;
  CharToFloat(buf+4+8, pressure);
  mps.pressure = pressure;
  CharToFloat(buf+4+12, humidity);
  mps.humidity = humidity;
  CharToDouble(buf+4+16, GPS_latitude);
  mps.GPS_latitude = GPS_latitude;
  CharToDouble(buf+4+24, GPS_longitude);
  mps.GPS_longitude = GPS_longitude;
}

void get_GPS(char* buf)
{
  double latitude, longitude, ext_height;
  int status = 0;
  CharToDouble(buf+3, latitude);
  gps.latitude = latitude;
  CharToDouble(buf+11, longitude);
  gps.longitude = longitude;
  gps.status.status = CharToInt(buf[19]);
  gps.altitude = CharToInt(buf[20]);
  gps.header.stamp = ros::Time::now();
  CharToDouble(buf+21, ext_height);
  height.range = ext_height;
  height.header.stamp = ros::Time::now();
}

void get_battery(char* buf)
{
  float voltage = 0;
  CharToFloat(buf+3, voltage);
  battery.voltage = voltage;
  battery.header.stamp = ros::Time::now();
}

void form_takeoff(char* buf, int agent_number, bool takeoff)
{
  buf[1] = IntToChar(agent_number);
  buf[2] = IntToChar(COMMAND_TAKEOFF);
  buf[3] = IntToChar(takeoff);
  buf[4] = 0x0A;
}

void form_waypoint_info(char* buf, int agent_number, int waypoint_number, enif_iuc::WaypointTask &waypoint_list)
{
  buf[1] = IntToChar(agent_number);
  buf[2] = IntToChar(COMMAND_WAYPOINT);
  buf[3] = IntToChar(waypoint_number);
  DoubleToChar(buf+4, waypoint_list.velocity);
  DoubleToChar(buf+12, waypoint_list.damping_distance);
}

void form_waypoints(char* buf, int waypoint_number, enif_iuc::WaypointTask &waypoint_list)
{
  int byte_number = 0;
  for(int i=0; i<waypoint_number; i++)
    {
      DoubleToChar(buf+20+byte_number, waypoint_list.mission_waypoint[i].latitude);
      byte_number += sizeof(double);
      DoubleToChar(buf+20+byte_number, waypoint_list.mission_waypoint[i].longitude);
      byte_number += sizeof(double);
      DoubleToChar(buf+20+byte_number, waypoint_list.mission_waypoint[i].target_height);
      byte_number += sizeof(double);
      buf[20+byte_number] = IntToChar(waypoint_list.mission_waypoint[i].staytime);
      byte_number++;
    }
  buf[20+byte_number] = 0x0A;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "enif_iuc_ground");
  ros::NodeHandle n;

  ros::Publisher  state_pub    = n.advertise<enif_iuc::AgentState>("agentState", 1);
  ros::Publisher  mps_pub      = n.advertise<enif_iuc::AgentMPS>("mps_data", 1);
  ros::Publisher  GPS_pub      = n.advertise<enif_iuc::AgentGlobalPosition>("global_position", 1);
  ros::Publisher  height_pub   = n.advertise<enif_iuc::AgentHeight>("ext_height", 1);
  ros::Publisher  battery_pub  = n.advertise<enif_iuc::AgentBatteryState>("battery", 1);
  ros::Subscriber sub_takeoff  = n.subscribe("takeoff_command",1,takeoff_callback);
  ros::Subscriber sub_wp       = n.subscribe("waypoint_list",1,wp_callback);
  
  ros::Rate loop_rate(100);

  struct timeval tvstart, tvend, timeout;
  gettimeofday(&tvstart,NULL);
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;

  // Start the USB serial port
  serial::Serial USBPORT("/dev/ttyUSB0", 9600, serial::Timeout::simpleTimeout(100));
  if(USBPORT.isOpen())
    cout<<"Wireless UART port opened"<<endl;
  else
    cout<<"Error opening Serial device. Check permission on reading serial port first!"<<endl;
  
  int count = 0, send_count = 0;
  int waypoint_number = 0;

  while (ros::ok())
  {
    char buf[256] = {'\0'};
    string data = USBPORT.read(256+1);
    strcpy(buf, data.c_str());
    // Get the target number first
    int target_number = get_target_number(buf);
    //cout<<"Target number: "<<target_number<<endl;
    if(target_number < 0){
      //Do nothing because GS need to subs to all other topics
      //cout<<"Error receiving data. Target number is negative: "<<target_number<<endl;
    }else{
      // Get command type
      int command_type = get_command_type(buf);
      enif_iuc::AgentGlobalPosition agent_gps;
      enif_iuc::AgentHeight agent_height;
      enif_iuc::AgentMPS agent_mps;
      enif_iuc::AgentState agent_state;
      enif_iuc::AgentBatteryState agent_battery;
      bool checksum_result = false;
      switch(command_type){
      case COMMAND_GPS:
	//form GPS, ext_height(lidar height) and publish
	get_GPS(buf);
	agent_gps.agent_number = target_number;
	agent_gps.gps = gps;
	GPS_pub.publish(agent_gps);
	agent_height.agent_number = target_number;
	agent_height.height = height;
	checksum_result = checksum(buf);
	if(checksum_result)
	  height_pub.publish(agent_height);
	break;
      case COMMAND_MPS:
	//form mps and publish
	get_mps(buf);
	agent_mps.agent_number = target_number;
	agent_mps.mps = mps;
	checksum_result = checksum(buf);
	if(checksum_result)
	  mps_pub.publish(agent_mps);
	break;
      case COMMAND_STATE:
	//form state and publish
	get_state(buf);
	agent_state.agent_number = target_number;
	agent_state.state = state;
	checksum_result = checksum(buf);
	if(checksum_result)
	  state_pub.publish(agent_state);
	break;
      case COMMAND_BATTERY:
	//form battery state and publish
	get_battery(buf);
	agent_battery.agent_number = target_number;
	agent_battery.battery = battery;
	checksum_result = checksum(buf);
	if(checksum_result)
	  battery_pub.publish(agent_battery);
	break;
      default:
	break;
      }
    }
    if(count%10 == 0)
      {
	char send_buf[256] = {'\0'};
	switch(send_count){
	case 0:
	  if(NEW_TAKEOFF)
	    {
	      form_takeoff(send_buf, agent_takeoff.agent_number, agent_takeoff.takeoff_command);
	      form_checksum(send_buf);
	      NEW_TAKEOFF = false;
	      string send_data(send_buf);
	      USBPORT.write(send_data);
	      cout<<send_data<<endl;
	    }
	  break;
	case 1:
	  if(NEW_WP)
	    {
	      form_waypoint_info(send_buf, agent_wp.agent_number, agent_wp.waypoint_list.mission_waypoint.size(), agent_wp.waypoint_list);
	      form_waypoints(send_buf, agent_wp.waypoint_list.mission_waypoint.size(), agent_wp.waypoint_list);
	      form_checksum(send_buf);
	      NEW_WP = false;
	      string send_data(send_buf);
	      USBPORT.write(send_data);
	    }
	  break;
	default:
	  break;
	}
	if(send_count < 1) send_count++;
	else send_count = 0;
      }
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
