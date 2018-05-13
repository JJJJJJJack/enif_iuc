#include "enif.h"

#include "enif_iuc/AgentTakeoff.h"
#include "enif_iuc/AgentWaypointTask.h"
#include "enif_iuc/AgentGlobalPosition.h"
#include "enif_iuc/AgentMPS.h"
#include "enif_iuc/AgentState.h"
#include "enif_iuc/AgentHeight.h"
#include "enif_iuc/AgentBatteryState.h"
#include "enif_iuc/AgentBox.h"

bool NEW_STATE = false, NEW_MPS = false, NEW_GPS = false, NEW_HEIGHT = false, NEW_BATTERY = false;

using namespace std;

void state_callback(const std_msgs::UInt8 &new_message)
{
  state = new_message;
  NEW_STATE = true;
}

void mps_callback(const mps_driver::MPS &new_message)
{
  mps = new_message;
  if(mps.gasID.compare("Propane")==0)
    GAS_ID = GAS_PROPANE;
  else if(mps.gasID.compare("Methane")==0)
    GAS_ID = GAS_METHANE;
  else
    GAS_ID = GAS_NONE;
  NEW_MPS = true;
}

void GPS_callback(const sensor_msgs::NavSatFix &new_message)
{
  gps = new_message;
  NEW_MPS = true;
}

void height_callback(const sensor_msgs::Range &new_message)
{
  height = new_message;
  NEW_HEIGHT = true;
}

void battery_callback(const sensor_msgs::BatteryState &new_message)
{
  battery = new_message;
  NEW_BATTERY = true;
}

void form_mps(char* buf)
{
  buf[1] = IntToChar(AGENT_NUMBER);
  buf[2] = IntToChar(COMMAND_MPS);
  buf[3] = IntToChar(GAS_ID);
  FloatToChar(buf+4, mps.percentLEL);
  FloatToChar(buf+4+4, mps.temperature);
  FloatToChar(buf+4+8, mps.pressure);
  FloatToChar(buf+4+12, mps.humidity);
  DoubleToChar(buf+4+16, gps.latitude);
  DoubleToChar(buf+4+24, gps.longitude);
  DoubleToChar(buf+4+32, height.range);
  buf[4+40] = 0x0A;
  // Clear the percentLEL to make sure we don't pub wrong data when we get new GPS
  clearmps();
}

void form_GPS(char* buf)
{
  buf[1] = IntToChar(AGENT_NUMBER);
  buf[2] = IntToChar(COMMAND_GPS);
  DoubleToChar(buf+3, gps.latitude);
  DoubleToChar(buf+11, gps.longitude);
  buf[19] = IntToChar(gps.status.status);
  buf[20] = IntToChar(gps.altitude);
  DoubleToChar(buf+21, height.range);
  buf[29] = 0x0A;
}

void form_state(char* buf)
{
  buf[1] = IntToChar(AGENT_NUMBER);
  buf[2] = IntToChar(COMMAND_STATE);
  buf[3] = IntToChar(state.data);
  buf[4] = 0x0A;
}

void form_battery(char* buf)
{
  buf[1] = IntToChar(AGENT_NUMBER);
  buf[2] = IntToChar(COMMAND_BATTERY);
  FloatToChar(buf+3, battery.voltage);
  buf[7] = 0x0A;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "enif_iuc_quad");
  ros::NodeHandle n;
  // Receive from Xbee and publish to local ROS
  ros::Publisher  takeoff_pub = n.advertise<std_msgs::Bool>("takeoff_command", 1);
  ros::Publisher  wp_pub      = n.advertise<enif_iuc::WaypointTask>("waypoint_list", 1);
  ros::Publisher  box_pub     = n.advertise<std_msgs::Float64MultiArray>("rotated_box", 1);
  ros::Publisher  mps_pub      = n.advertise<enif_iuc::AgentMPS>("agent_mps_data", 1);
  ros::Publisher  GPS_pub      = n.advertise<enif_iuc::AgentGlobalPosition>("agent_global_position", 1);
  // Subscribe topics from onboard ROS and transmit it through Xbee
  ros::Subscriber sub_state   = n.subscribe("agentState",1,state_callback);
  ros::Subscriber sub_mps     = n.subscribe("mps_data",1,mps_callback);
  ros::Subscriber sub_GPS     = n.subscribe("mavros/global_position/global",1,GPS_callback);
  ros::Subscriber sub_height  = n.subscribe("mavros/distance_sensor/lidarlite_pub",1,height_callback);
  ros::Subscriber sub_battery = n.subscribe("mavros/battery",1,battery_callback);

  n.getParam("/enif_iuc_quad/AGENT_NUMBER", AGENT_NUMBER);
  cout<<"This is Agent No."<<AGENT_NUMBER<<endl;
  
  ros::Rate loop_rate(100);

  struct timeval tvstart, tvend, timeout;
  gettimeofday(&tvstart,NULL);
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  // Start the USB serial port
  n.getParam("/enif_iuc_quad/USB", USB);
  serial::Serial USBPORT(USB, 9600, serial::Timeout::simpleTimeout(100));
  
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
    if(target_number != AGENT_NUMBER){
      if(target_number > 0){
	// Get command type
	cout<<"Receiving other quad info: ";
	int command_type = get_command_type(buf);
	bool checksum_result = false;
	enif_iuc::AgentMPS agent_mps;
	enif_iuc::AgentGlobalPosition agent_gps;	      
	sensor_msgs::NavSatFix my_gps = gps;
	mps_driver::MPS my_mps = mps;
	cout<<"command type: "<<command_type<<endl;
	switch(command_type){
	case COMMAND_MPS:
	  //form mps and publish
	  get_mps(buf);
	  agent_mps.agent_number = target_number;
	  agent_mps.mps = mps;
	  checksum_result = checksum(buf);
	  if(checksum_result)
	    if(mps.percentLEL != 0)
	      mps_pub.publish(agent_mps);
	    else{
	      agent_gps.agent_number = target_number;
	      extract_GPS_from_MPS(mps);
	      agent_gps.gps = gps;
	      GPS_pub.publish(agent_gps);
	    }
	  if(NEW_MPS || NEW_GPS){
	    mps = my_mps; gps = my_gps;
	  }
	  cout<<" Info from agent."<<target_number<<endl;
	  break;
	default:
	  break;
	}
      }
      //Do nothing if target number doesn't match
    }else{
      // Get command type
      cout<<"Receiving command: ";
      int command_type = get_command_type(buf);
      bool checksum_result = false;
      switch(command_type){
      case COMMAND_WAYPOINT:{
	// Publish waypoint
	cout<< " Waypoint"<<endl;
	waypoint_list.mission_waypoint.clear();
	waypoint_number = get_waypoint_number(buf);
	get_waypoint_info(buf, waypoint_list);
	get_waypoints(waypoint_number, buf, waypoint_list);
	checksum_result = checksum(buf);
	cout<<waypoint_list<<endl;
	string send_data(buf);
	USBPORT.write(send_data);
	//if(checksum_result)
	//  wp_pub.publish(waypoint_list);
	break;
      }
      case COMMAND_BOX:{
	cout<< " Box"<<endl;
	box.data.clear();
	get_box(buf, box);
	checksum_result = checksum(buf);
	cout<<box<<endl;
	string send_data(buf);
	USBPORT.write(send_data);
	//if(checksum_result)
	//  wp_pub.publish(waypoint_list);
	break;

      }
      case COMMAND_TAKEOFF:{
	std_msgs::Bool cmd = get_takeoff_command(buf);
	takeoff_command.data = cmd.data;
	if(takeoff_command.data == true)
	  cout<< " Takeoff"<<endl;
	else
	  cout<< " Land"<<endl;
	checksum_result = checksum(buf);
	string send_data(buf);
	USBPORT.write(send_data);
	//string send_data;
	//if(checksum_result)
	//  takeoff_pub.publish(takeoff_command);
	break;
      }
      case COMMAND_GPS:{
	cout<<"my own GPS"<<endl;
	break;
      }
      default:
	break;
      }
      if(checksum_result)
	{
	  wp_pub.publish(waypoint_list);
	  takeoff_pub.publish(takeoff_command);
	  box_pub.publish(box);
	}
    }
    // Send GPS mps state and battery data every 1 sec
    if(count%5 == 0)
      {
	char send_buf[256] = {'\0'};
	switch(send_count){
	case 0:
	  if(NEW_MPS){
	    form_mps(send_buf);
	    form_checksum(send_buf);
	    string send_data(send_buf);
	    USBPORT.write(send_data);
	    NEW_MPS = false;
	  }
	  break;
	case 1:
	  if(NEW_GPS){
	    form_GPS(send_buf);
	    form_checksum(send_buf);
	    string send_data(send_buf);
	    USBPORT.write(send_data);
	    NEW_GPS = false;
	  }
	  break;
	case 2:
	  if(NEW_STATE){
	    form_state(send_buf);
	    form_checksum(send_buf);
	    string send_data(send_buf);
	    USBPORT.write(send_data);
	    NEW_STATE = false;
	  }
	  break;
	case 3:
	  if(NEW_BATTERY){
	    form_battery(send_buf);
	    form_checksum(send_buf);
	    string send_data(send_buf);
	    USBPORT.write(send_data);
	    NEW_BATTERY = false;
	  }
	  break;
	default:
	  break;
	}
	if(send_count < 3) send_count++;
	else send_count = 0;
      }
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;

  }
  return 0;
}
  
