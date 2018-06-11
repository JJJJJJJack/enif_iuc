#include "enif.h"

#include "enif_iuc/AgentTakeoff.h"
#include "enif_iuc/AgentWaypointTask.h"
#include "enif_iuc/AgentGlobalPosition.h"
#include "enif_iuc/AgentMPS.h"
#include "enif_iuc/AgentState.h"
#include "enif_iuc/AgentHeight.h"
#include "enif_iuc/AgentBatteryState.h"
#include "enif_iuc/AgentBox.h"

bool NEW_TAKEOFF = false, NEW_WP = false, NEW_BOX = false;
enif_iuc::AgentTakeoff agent_takeoff[256];
enif_iuc::AgentWaypointTask agent_wp[256];
enif_iuc::AgentBox agent_box[256];
int recHome = 0;
int agent_number_wp = 0, agent_number_box = 0, agent_number_takeoff = 0;
bool waypoint_checked[256] = {true, true, true, true, true};
bool box_checked[256] = {true, true, true, true, true};
bool takeoff_checked[256] = {true, true, true, true, true};

using namespace std;

bool check_takeoff(std_msgs::Bool sendtakeoff, std_msgs::Bool responsetakeoff)
{
  bool result = false;
  if(sendtakeoff.data != responsetakeoff.data)
    return false;
  return true;
}

bool check_waypoints(enif_iuc::WaypointTask sendwp, enif_iuc::WaypointTask responsewp)
{
  bool result = false;

  if (sendwp.velocity != responsewp.velocity || sendwp.damping_distance != responsewp.damping_distance)
    return false;
  else
    if(sendwp.mission_waypoint.size() != responsewp.mission_waypoint.size())
      return false;
    else
      for(int i = 0; i < sendwp.mission_waypoint.size(); i++)
	{
	  if(sendwp.mission_waypoint[i].latitude != responsewp.mission_waypoint[i].latitude)
	    return false;
	  else
	    if(sendwp.mission_waypoint[i].longitude != responsewp.mission_waypoint[i].longitude)
	      return false;
	    else
	      if(sendwp.mission_waypoint[i].target_height != responsewp.mission_waypoint[i].target_height)
		return false;
	      else
		if(sendwp.mission_waypoint[i].staytime != responsewp.mission_waypoint[i].staytime)
		  return false;
	}
  return true;
}

bool check_box(std_msgs::Float64MultiArray sendbox, std_msgs::Float64MultiArray responsebox)
{
  bool result = false;
  if(sendbox.data.size() != responsebox.data.size())
    return false;
  for(int i = 0; i < 7; i++){
    if(sendbox.data[i]!=responsebox.data[i])
      return false;
  }
  return true;
}

void takeoff_callback(const enif_iuc::AgentTakeoff &new_message)
{
  agent_number_takeoff = new_message.agent_number;
  bool check_result = check_takeoff(new_message.takeoff_command, agent_takeoff[agent_number_takeoff].takeoff_command);
  if(check_result == false)
    {
      agent_takeoff[agent_number_takeoff] = new_message;
      takeoff_checked[agent_number_takeoff] = false;
    }
}

void wp_callback(const enif_iuc::AgentWaypointTask &new_message)
{
  agent_number_wp = new_message.agent_number;
  bool check_result = check_waypoints(new_message.waypoint_list, agent_wp[agent_number_wp].waypoint_list);
  if(check_result == false)// overwrite when new waypoints coming in
    {
      agent_wp[agent_number_wp] = new_message;
      waypoint_checked[agent_number_wp] = false;
    }
}

void box_callback(const enif_iuc::AgentBox &new_message)
{
  agent_number_box = new_message.agent_number;
  bool check_result = check_box(new_message.box, agent_box[agent_number_box].box);
  if(check_result == false)// overwrite when new box coming in
    {
      agent_box[agent_number_box] = new_message;
      box_checked[agent_number_box] = false;
    }
}

void get_state(char* buf)
{
  state.data = CharToInt(buf[3]);
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

void form_takeoff(char* buf, int agent_number, std_msgs::Bool takeoff)
{
  buf[1] = IntToChar(agent_number);
  buf[2] = IntToChar(COMMAND_TAKEOFF);
  buf[3] = IntToChar(takeoff.data);
  buf[4] = 0x0A;
}

void form_home(char* buf, int agent_number, geographic_msgs::GeoPoint aveHome)
{
  // different than enif_iuc_quad
  buf[1] = IntToChar(agent_number);
  buf[2] = IntToChar(COMMAND_AVEHOME);
  DoubleToChar(buf+3, aveHome.latitude);
  DoubleToChar(buf+11, aveHome.longitude);
  FloatToChar(buf+19, aveHome.altitude);
  buf[19+4] = 0x0A;
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

void form_box(char* buf, int agent_number, std_msgs::Float64MultiArray &box)
{
  buf[1] = IntToChar(agent_number);
  buf[2] = IntToChar(COMMAND_BOX);
  DoubleToChar(buf+3,  box.data[0]);
  DoubleToChar(buf+11, box.data[1]);
  DoubleToChar(buf+19, box.data[2]);
  DoubleToChar(buf+27, box.data[3]);
  DoubleToChar(buf+35, box.data[4]);
  buf[43] = IntToChar((int)box.data[5]);
  buf[44] = IntToChar((int)box.data[6]);
  buf[45] = IntToChar((int)box.data[7]);
  buf[46] = IntToChar((int)box.data[8]);
  buf[47] = 0x0A;
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
  ros::Subscriber sub_takeoff  = n.subscribe("takeoff_command",5,takeoff_callback);
  ros::Subscriber sub_wp       = n.subscribe("waypoint_list",5,wp_callback);
  ros::Subscriber sub_box      = n.subscribe("rotated_box",1,box_callback);
  
  ros::Rate loop_rate(100);

  struct timeval tvstart, tvend, timeout;
  gettimeofday(&tvstart,NULL);
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;

  // Start the USB serial port
  n.getParam("/enif_iuc_ground/USB", USB);
  serial::Serial USBPORT(USB, 9600, serial::Timeout::simpleTimeout(100));
  
  // Start the USB serial port
  if(USBPORT.isOpen())
    cout<<"Wireless UART port opened"<<endl;
  else
    cout<<"Error opening Serial device. Check permission on reading serial port first!"<<endl;
  
  int count = 0, send_count = 0;
  int waypoint_number = 0, response_number = 0;

  while (ros::ok())
  {
    char charbuf[256] = {'\0'};
    string data = USBPORT.read(256+1);
    strcpy(charbuf, data.c_str());
    char* buf = charbuf;
    // Get command type
    int command_type = get_command_type(buf);
    int c=0;
    while(buf[0]!='\0' && (command_type==COMMAND_MPS || command_type==COMMAND_HOME || command_type==COMMAND_LOCAL || command_type==COMMAND_STATE || command_type==COMMAND_WAYPOINT || command_type==COMMAND_BOX)){
      //cout<<strlen(buf)<<endl;
    // Get the target number first
    int target_number = get_target_number(buf);
    command_type = get_command_type(buf);
    //cout<<"Target number: "<<target_number<<endl;
    if(target_number < 0){
      //Do nothing because GS need to subs to all other topics
      //cout<<"Error receiving data. Target number is negative: "<<target_number<<endl;
    }else{
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
	agent_height.agent_number = target_number;
	agent_height.height = height;
	checksum_result = checksum(buf);
	if(checksum_result && agent_gps.gps.status.status == 0){
	  height_pub.publish(agent_height);
	  GPS_pub.publish(agent_gps);
	}
	break;
      case COMMAND_MPS:
	//form mps and publish
	checksum_result = checksum(buf);
	get_mps(buf);
	ROS_INFO_THROTTLE(0.1, "Quad No.%d MPS", target_number);
	agent_mps.agent_number = target_number;
	agent_mps.mps = mps;
	buf = buf+45;
	if(mps.percentLEL != 0)
	  mps_pub.publish(agent_mps);
	else{
	  agent_gps.agent_number = target_number;
	  if(extract_GPS_from_MPS(mps) == true){
	    agent_gps.gps = gps;
	    agent_height.agent_number = target_number;
	    agent_height.height = height;
	    height_pub.publish(agent_height);
	    GPS_pub.publish(agent_gps);
	  }
	}
	break;
      case COMMAND_STATE:
	//form state and publish
	checksum_result = checksum(buf);
	get_state(buf);
	agent_state.agent_number = target_number;
	agent_state.state = state;
	ROS_INFO_THROTTLE(0.1, "Quad No.%d state", target_number);
	buf = buf+5;
	state_pub.publish(agent_state);
	break;
      case COMMAND_BATTERY:
	//form battery state and publish
	checksum_result = checksum(buf);
	get_battery(buf);
	agent_battery.agent_number = target_number;
	agent_battery.battery = battery;
	buf = buf+8;
	battery_pub.publish(agent_battery);
	break;
      case COMMAND_WAYPOINT:
	//response from the agent
	waypoint_list.mission_waypoint.clear();
	response_number = get_target_number(buf);
	waypoint_number = get_waypoint_number(buf);
	get_waypoint_info(buf, waypoint_list);
	get_waypoints(waypoint_number, buf, waypoint_list);
	waypoint_checked[response_number] = check_waypoints(agent_wp[response_number].waypoint_list, waypoint_list);
	cout<<"Waypoint check: "<<waypoint_checked[response_number]<<endl;
	cout<<waypoint_list<<endl;
	buf = buf+get_waypointlist_buf_size(waypoint_number)+1;
	break;
      case COMMAND_BOX:
	//only verifies response from the agent
	response_number = get_target_number(buf);
	box.data.clear();
	get_box(buf, box);
	box_checked[response_number] = check_box(agent_box[response_number].box, box);
	cout<<"Box check: "<<box_checked[response_number]<<endl;
	cout<<box<<endl;
	buf = buf+48;
	break;
      case COMMAND_TAKEOFF:
	//only verifies response from the agent
	response_number = get_target_number(buf);
	takeoff_command = get_takeoff_command(buf);
	takeoff_checked[response_number] = check_takeoff(agent_takeoff[response_number].takeoff_command, takeoff_command);
	cout<<"Takeoff check: "<<takeoff_checked[response_number]<<endl;
	cout<<takeoff_command<<endl;
	buf = buf+5;
	break;
      case COMMAND_LOCAL:
	buf = buf+60;
	break;
      case COMMAND_HOME:
	response_number = get_target_number(buf);	
	get_home(buf);
	if (checkHome(home)){	 	  
	  std::vector<int>::iterator it;
	  it = std::find(agentID.begin(), agentID.end(), response_number);
	  if (it != agentID.end()){ // if i already have gps    
	    // overwrite it
	    int i = it - agentID.begin();
	    agentHomes[i] = home.geo;
	  }
	  else{
	    // push new home back        	
	    agentHomes.push_back(home.geo);
	    agentID.push_back(response_number);
	  }
	  cout<<"Recieved agent "<< response_number<<"'s home: "<<home.geo.latitude<<", "<<home.geo.longitude<<", "<<home.geo.altitude<<endl;
	  recHome += 1;
	}
	buf = buf+24;
	break;
      default:
	break;
      }
    }
    if(c++>50)
      break;
    }
    if(count%3 == 0)
      {
	char send_buf[256] = {'\0'};
	switch(send_count){
	case 0:
	  if(takeoff_checked[agent_number_takeoff] == false && agent_number_takeoff > 0)
	    {
	      form_takeoff(send_buf, agent_takeoff[agent_number_takeoff].agent_number, agent_takeoff[agent_number_takeoff].takeoff_command);
	      form_checksum(send_buf);
	      NEW_TAKEOFF = false;
	      string send_data(send_buf);
	      USBPORT.write(send_data);
	      cout<<"Send takeoff command to agent "<<agent_number_takeoff<<endl;
	    }
	  break;
	case 1:
	  // Box has higher priority
	  if(box_checked[agent_number_box] == false && agent_number_box > 0)
	    {
	      form_box(send_buf, agent_box[agent_number_box].agent_number, agent_box[agent_number_box].box);
	      form_checksum(send_buf);
	      NEW_BOX = false;
	      string send_data(send_buf);
	      USBPORT.write(send_data);
	      cout<<"send box to agent "<<agent_number_box<<endl;
	      cout<<agent_box[agent_number_box].box<<endl;
	    }
	  else if(waypoint_checked[agent_number_wp] == false && agent_number_wp > 0)
	    {
	      form_waypoint_info(send_buf, agent_wp[agent_number_wp].agent_number, agent_wp[agent_number_wp].waypoint_list.mission_waypoint.size(), agent_wp[agent_number_wp].waypoint_list);
	      form_waypoints(send_buf, agent_wp[agent_number_wp].waypoint_list.mission_waypoint.size(), agent_wp[agent_number_wp].waypoint_list);
	      form_checksum(send_buf);
	      NEW_WP = false;
	      string send_data(send_buf);
	      USBPORT.write(send_data);
	      cout<<"send waypoint to agent "<<agent_number_wp<<endl;
	    }
	  break;
	case 2:
	  //send ave home location to all agents
	  if (recHome>4 && agentHomes.size()!= 0){
	    //cout<<"num of agent homes: "<<agentHomes.size()<<endl;
	    getAvehome();
	    form_home(send_buf, 100, averageHome); // sending as agent_number: 100 to recieve on quad side
	    form_checksum(send_buf);
	    string send_data(send_buf);
	    USBPORT.write(send_data);
	    cout<<"send ave home to all agents"<<endl;
	    // ros::Duration(0.5).sleep();
	  }		  
	  break;	  
	default:
	  break;
	}
	if(send_count < 2) send_count++;
	else send_count = 0;
      }
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
