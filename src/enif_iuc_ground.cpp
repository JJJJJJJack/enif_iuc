#include "enif.h"

#include "enif_iuc/AgentTakeoff.h"
#include "enif_iuc/AgentWaypointTask.h"
#include "enif_iuc/AgentGlobalPosition.h"
#include "enif_iuc/AgentMPS.h"
#include "enif_iuc/AgentState.h"
#include "enif_iuc/AgentHeight.h"
#include "enif_iuc/AgentBatteryState.h"
#include "enif_iuc/AgentBox.h"
#include "enif_iuc/AgentCheck.h"
#include "enif_iuc/AgentSource.h"

enif_iuc::AgentTakeoff agent_takeoff[256];
enif_iuc::AgentWaypointTask agent_wp[256];
enif_iuc::AgentBox agent_box[256];
enif_iuc::AgentCheck wpcheck_msg;
enif_iuc::AgentCheck boxcheck_msg;
enif_iuc::AgentCheck sourcecheck_msg;
enif_iuc::AgentSource agent_source[256];

int recHome = 0;
int agent_number_wp = 0, agent_number_box = 0, agent_number_takeoff = 0, agent_number_source = 0;
bool waypoint_checked[256] = {true, true, true, true, true};
bool box_checked[256] = {true, true, true, true, true};
bool takeoff_checked[256] = {true, true, true, true, true};
bool source_checked[256] = {true, true, true, true, true};

serial::Serial USBPORT("/dev/xbee", 9600, serial::Timeout::simpleTimeout(100));

using namespace std;

bool check_takeoff(std_msgs::Bool sendtakeoff, std_msgs::Bool responsetakeoff)
{
  bool result = false;
  if(sendtakeoff.data != responsetakeoff.data)
    return false;
  return true;
}

void form_realTarget(char* buf, int agent_number, enif_iuc::AgentSource sendSource)
{
  buf[2] = IntToChar(agent_number);
  buf[3] = IntToChar(COMMAND_REALTARGET);  
  DoubleToChar(buf+4, sendSource.source.latitude);
  DoubleToChar(buf+12, sendSource.source.longitude);
  DoubleToChar(buf+20, sendSource.source.altitude);
  FloatToChar(buf+28, sendSource.angle);
  FloatToChar(buf+32, sendSource.wind_speed);
  FloatToChar(buf+36, sendSource.diff_y);
  FloatToChar(buf+40,sendSource.diff_z);
  FloatToChar(buf+44,sendSource.release_rate);
  buf[48] = 0x0A;
}

void form_takeoff(char* buf, int agent_number, std_msgs::Bool takeoff)
{
  buf[2] = IntToChar(agent_number);
  buf[3] = IntToChar(COMMAND_TAKEOFF);
  buf[4] = IntToChar(takeoff.data*100 + alg.data);
  buf[5] = 0x0A;
}

void form_waypoint_info(char* buf, int agent_number, int waypoint_number, enif_iuc::WaypointTask &waypoint_list)
{
  buf[2] = IntToChar(agent_number);
  buf[3] = IntToChar(COMMAND_WAYPOINT);
  buf[4] = IntToChar(waypoint_number);
  DoubleToChar(buf+5, waypoint_list.velocity);
  DoubleToChar(buf+13, waypoint_list.damping_distance);
}

void form_waypoints(char* buf, int waypoint_number, enif_iuc::WaypointTask &waypoint_list)
{
  int byte_number = 0;
  for(int i=0; i<waypoint_number; i++)
    {
      DoubleToChar(buf+21+byte_number, waypoint_list.mission_waypoint[i].latitude);
      byte_number += sizeof(double);
      DoubleToChar(buf+21+byte_number, waypoint_list.mission_waypoint[i].longitude);
      byte_number += sizeof(double);
      DoubleToChar(buf+21+byte_number, waypoint_list.mission_waypoint[i].target_height);
      byte_number += sizeof(double);
      buf[21+byte_number] = IntToChar(waypoint_list.mission_waypoint[i].staytime);
      byte_number++;
    }
  buf[21+byte_number] = 0x0A;
}

void form_box(char* buf, int agent_number, std_msgs::Float64MultiArray &box)
{
  buf[2] = IntToChar(agent_number);
  buf[3] = IntToChar(COMMAND_BOX);
  DoubleToChar(buf+4,  box.data[0]);
  DoubleToChar(buf+12, box.data[1]);
  DoubleToChar(buf+20, box.data[2]);
  DoubleToChar(buf+28, box.data[3]);
  DoubleToChar(buf+36, box.data[4]);
  buf[44] = IntToChar((int)box.data[5]);
  buf[45] = IntToChar((int)(box.data[6]*20.0));
  buf[46] = IntToChar((int)box.data[7]);
  buf[47] = IntToChar((int)box.data[8]);
  buf[48] = IntToChar((int)box.data[9]);
  buf[49] = IntToChar((int)box.data[10]);  
  buf[50] = 0x0A;
}

bool check_alg(std_msgs::Int8 sendAlg, std_msgs::Int8 responseAlg)
{
  bool result = false;
  if(sendAlg.data != responseAlg.data)
    return false;
  return true;
}

bool check_source(enif_iuc::AgentSource sendsource, enif_iuc::AgentSource responsesource)
{
  if(sendsource.source.latitude != responsesource.source.latitude || sendsource.source.longitude != responsesource.source.longitude || sendsource.source.altitude != responsesource.source.altitude || sendsource.angle != responsesource.angle || sendsource.diff_y != responsesource.diff_y || sendsource.release_rate != responsesource.release_rate){
    cout<<sendsource.source.latitude - responsesource.source.latitude<<endl;
    cout<<sendsource.source.longitude - responsesource.source.longitude<<endl;
    cout<<sendsource.source.altitude - responsesource.source.altitude<<endl;
    return false;
  }
  return true;
}

bool check_return_source(enif_iuc::AgentSource sendsource, enif_iuc::AgentSource responsesource)
{
  if(fabs(sendsource.source.latitude - responsesource.source.latitude)>1e-04 || fabs(sendsource.source.longitude - responsesource.source.longitude)>1e-04 || fabs(sendsource.source.altitude - responsesource.source.altitude)>1e-04 || fabs(sendsource.angle - responsesource.angle)>1e-04 || fabs(sendsource.diff_y - responsesource.diff_y)>1e-04 || fabs(sendsource.release_rate - responsesource.release_rate)>1e-04){
    cout<<sendsource.source.latitude - responsesource.source.latitude<<endl;
    cout<<sendsource.source.longitude - responsesource.source.longitude<<endl;
    cout<<sendsource.source.altitude - responsesource.source.altitude<<endl;
    return false;
  }
  return true;
}


bool check_waypoints(enif_iuc::WaypointTask sendwp, enif_iuc::WaypointTask responsewp)
{
  bool result = false;
  // Return false if there's anything different
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

bool check_return_waypoints(enif_iuc::WaypointTask sendwp, enif_iuc::WaypointTask responsewp)
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
	  if(fabs(sendwp.mission_waypoint[i].latitude - responsewp.mission_waypoint[i].latitude)>1e-04)
	    return false;
	  else
	    if(fabs(sendwp.mission_waypoint[i].longitude - responsewp.mission_waypoint[i].longitude)>1e-04)
	      return false;
	    else
	      if(fabs(sendwp.mission_waypoint[i].target_height - responsewp.mission_waypoint[i].target_height)>1e-04)
		return false;
	      else
		if(fabs(sendwp.mission_waypoint[i].staytime - responsewp.mission_waypoint[i].staytime)>1e-04)
		  return false;
	}
  return true;
}


bool check_box(std_msgs::Float64MultiArray sendbox, std_msgs::Float64MultiArray responsebox)
{
  bool result = false;
  if(sendbox.data.size() != responsebox.data.size())
    return false;
  for(int i = 0; i < 11; i++){
    if(sendbox.data[i]!=responsebox.data[i])
      return false;
  }
  return true;
}

bool check_return_box(std_msgs::Float64MultiArray sendbox, std_msgs::Float64MultiArray responsebox)
{
  bool result = false;
  if(sendbox.data.size() != responsebox.data.size())
    return false;
  for(int i = 0; i < 11; i++){
    if(fabs(sendbox.data[i] - responsebox.data[i]) > 1e-04)
      return false;
  }
  return true;
}

void alg_callback(const std_msgs::Int8 &new_message)
{
  bool check_result = check_alg(new_message, alg);
  if(check_result == false)
    {
      for(int i = 0; i < 256; i++){
	takeoff_checked[i] = false;
      }
    }
  alg = new_message;
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

  if(takeoff_checked[agent_number_takeoff] == false && agent_number_takeoff > 0)
    {
      char send_buf[256] = {'\0'};
      form_start(send_buf);
      form_takeoff(send_buf, agent_number_takeoff, agent_takeoff[agent_number_takeoff].takeoff_command);
      string send_data(send_buf);
      USBPORT.write(send_data);
      cout<<"Send takoff command to agent "<<agent_number_takeoff<<endl;
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
  
  if(waypoint_checked[agent_number_wp] == false && agent_number_wp > 0)
    {
      char send_buf[256] = {'\0'};
      form_start(send_buf);

      form_waypoint_info(send_buf, agent_wp[agent_number_wp].agent_number, agent_wp[agent_number_wp].waypoint_list.mission_waypoint.size(), agent_wp[agent_number_wp].waypoint_list);
      form_waypoints(send_buf, agent_wp[agent_number_wp].waypoint_list.mission_waypoint.size(), agent_wp[agent_number_wp].waypoint_list);

      string send_data(send_buf);
      USBPORT.write(send_data);
      cout<<"send waypoint to agent "<<agent_number_wp<<endl;
    }
  
}

void realTarget_callback(const enif_iuc::AgentSource &new_message){

  agent_number_source = new_message.agent_number;  
  bool check_result = check_source(new_message,agent_source[agent_number_source]);

  if (check_result==false){// overwrite when new source coming in
    agent_source[agent_number_source] = new_message;
    source_checked[agent_number_source] = false;
  }

  if(source_checked[agent_number_source] == false && agent_number_source > 0)
    {
      char send_buf[256] = {'\0'};
      form_start(send_buf);

      form_realTarget(send_buf, 100, agent_source[agent_number_source]); // broadcast
      string send_data(send_buf);
      USBPORT.write(send_data);
      cout<<"Send source command to agent "<<agent_number_source<<endl;
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

  if(box_checked[agent_number_box] == false && agent_number_box > 0)
    {
      char send_buf[256] = {'\0'};
      form_start(send_buf);  
      form_box(send_buf, agent_box[agent_number_box].agent_number, agent_box[agent_number_box].box);

      string send_data(send_buf);
      USBPORT.write(send_data);
      cout<<"send box to agent "<<agent_number_box<<endl;
      cout<<agent_box[agent_number_box].box<<endl;
    }

}

void get_state(char* buf)
{  
  state.data = CharToInt(buf[0]);
  package_length = 5;
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

void form_home(char* buf, int agent_number, geographic_msgs::GeoPoint aveHome)
{
  // different than enif_iuc_quad
  buf[1] = IntToChar(agent_number);
  buf[2] = IntToChar(COMMAND_AVEHOME);
  DoubleToChar(buf+3, aveHome.latitude);
  DoubleToChar(buf+11, aveHome.longitude);
  DoubleToChar(buf+19, aveHome.altitude);  
  buf[19+8] = 0x0A;  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "enif_iuc_ground");
  ros::NodeHandle n;

  ros::Publisher  state_pub    = n.advertise<enif_iuc::AgentState>("agentState", 1);
  ros::Publisher  mps_pub        = n.advertise<enif_iuc::AgentMPS>("mps_data", 1);  
  ros::Publisher  agentHomes_pub = n.advertise<geographic_msgs::GeoPoint>("agent_home", 1);  
  ros::Publisher  GPS_pub      = n.advertise<enif_iuc::AgentGlobalPosition>("global_position", 1);
  ros::Publisher  height_pub   = n.advertise<enif_iuc::AgentHeight>("ext_height", 5);
  ros::Publisher  battery_pub  = n.advertise<enif_iuc::AgentBatteryState>("battery", 5);
  ros::Publisher  wpcheck_pub  = n.advertise<enif_iuc::AgentCheck>("waypoint_check", 5);
  ros::Publisher  boxcheck_pub = n.advertise<enif_iuc::AgentCheck>("rotated_box_check", 5);
  ros::Publisher  agent_targetE_pub  = n.advertise<enif_iuc::AgentSource>("agent_targetE", 1);
  ros::Publisher  sourcecheck_pub = n.advertise<enif_iuc::AgentCheck>("source_check", 1);
  
  ros::Subscriber sub_takeoff    = n.subscribe("takeoff_command",5,takeoff_callback);
  ros::Subscriber sub_wp         = n.subscribe("waypoint_list",5,wp_callback);
  ros::Subscriber sub_box        = n.subscribe("rotated_box",5,box_callback);
  ros::Subscriber sub_realTarget = n.subscribe("source",5,realTarget_callback);
  ros::Subscriber sub_planningalgorithm = n.subscribe("planning_algorithm",5, alg_callback);

  ros::Rate loop_rate(100);

  struct timeval tvstart, tvend, timeout;
  gettimeofday(&tvstart,NULL);
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;

  // Start the USB serial port
  //  n.getParam("/enif_iuc_ground/USB", USB);

  
  // Start the USB serial port
  if(USBPORT.isOpen())
    cout<<"Wireless UART port opened"<<endl;
  else
    cout<<"Error opening Serial device. Check permission on reading serial port first!"<<endl;
  
  int count = 0, send_count = 0;
  int waypoint_number = 0, response_number = 0;

  while (ros::ok())
  {

    // check for start
    data = USBPORT.read(1);
    if(!data.compare("<")){
      data = USBPORT.read(1);
      if(!data.compare("<")){
	
	// process data	  
	data = USBPORT.read(2);	  
	char Tcharbuf[4] = {'\0'};
	char *Tbuf = Tcharbuf;
	strcpy(Tcharbuf, data.c_str());

	int command_type = get_command_type(Tbuf);
	int target_number = get_target_number(Tbuf);

	enif_iuc::AgentHeight agent_height;
	enif_iuc::AgentBatteryState agent_battery;
	std_msgs::Int8 response_alg;
      
	bool checksum_result = false;
	switch(command_type){
	case COMMAND_MPS:
	  {
	    USBPORT.setTimeout(serial::Timeout::max(),100,0,100,0);// adjust timeout

	    char charbuf[256] = {'\0'};
	    string command_data = USBPORT.read(MPS_LENGTH);
	    strcpy(charbuf, command_data.c_str());
	    char *buf = charbuf;
	    
	    ROS_INFO_THROTTLE(1,"Receiving mps quad info from Agent %d", target_number);

	    if (checkEnd(buf, MPS_LENGTH-1)){
	      get_mps(buf);
	      if(mps.percentLEL!=0 && mps.percentLEL>0 && mps.percentLEL<100){
		enif_iuc::AgentMPS agent_mps;
		agent_mps.agent_number = target_number;
		agent_mps.mps = mps;
		mps_pub.publish(agent_mps);
	      }
	      if(extract_GPS_from_MPS(mps) == true){
		enif_iuc::AgentGlobalPosition agent_gps;
		agent_gps.agent_number = target_number;
		agent_gps.gps = gps;
		agent_height.agent_number = target_number;
		agent_height.height = height;
		height_pub.publish(agent_height);
		GPS_pub.publish(agent_gps);
	      }
	    }
	    else{
	      ROS_INFO_THROTTLE(1,"no end data");
	    }
	  }
	  break;
	case COMMAND_STATE:
	  {
	    //form state and publish
	  
	    USBPORT.setTimeout(serial::Timeout::max(),100,0,100,0);// adjust timeout
	    char charbuf[256] = {'\0'};
	    string command_data = USBPORT.read(STATE_LENGTH);
	    strcpy(charbuf, command_data.c_str());
	    char *buf = charbuf;

	    ROS_INFO_THROTTLE(1,"Receiving state info from Agent %d", target_number);
	    if (checkEnd(buf, STATE_LENGTH-1)){
	      enif_iuc::AgentState agent_state;
	      get_state(buf);
	      agent_state.agent_number = target_number;
	      agent_state.state = state;
	      state_pub.publish(agent_state);
	    }
	    else{
	      ROS_INFO_THROTTLE(1,"no end data");
	    }

	  }
	  break;
	case COMMAND_WAYPOINT:
	  {
	    //response from the agent
	    USBPORT.setTimeout(serial::Timeout::max(),1000,0,1000,0);// adjust timeout
	    char Wcharbuf[256] = {'\0'};
	    string Wcommand_data = USBPORT.read(1); // get number of waypoints
	    strcpy(Wcharbuf, Wcommand_data.c_str());
	    char *Wbuf = Wcharbuf;
	    waypoint_number = get_waypoint_number(Wbuf);
	    int numBytes = get_waypointlist_buf_size(waypoint_number)+1;

	    char charbuf[256] = {'\0'};
	    string command_data = USBPORT.read(numBytes); 
	    strcpy(charbuf, command_data.c_str());
	    char *buf = charbuf;

	    cout<<"waypoints from"<<target_number<<endl;

	    if (checkEnd(buf,numBytes-1)){
	      waypoint_list.mission_waypoint.clear();
	      get_waypoint_info(buf, waypoint_list);
	      get_waypoints(waypoint_number, buf, waypoint_list);
	      cout<<waypoint_list<<endl;

	      response_number = target_number;	    
	      waypoint_checked[response_number] = check_return_waypoints(agent_wp[response_number].waypoint_list, waypoint_list);
	      cout<<"Waypoint check: "<<waypoint_checked[response_number]<<endl;
	      wpcheck_msg.agent_number = response_number;
	      wpcheck_msg.check.data = waypoint_checked[response_number];
	      wpcheck_pub.publish(wpcheck_msg);
	      cout<<agent_wp[response_number]<<endl;
	    }
	  }
	  break;
	case COMMAND_BOX:
	  {
	    //only verifies response from the agent

	    USBPORT.setTimeout(serial::Timeout::max(),150,0,150,0);// adjust timeout
	    char charbuf[256] = {'\0'};
	    string command_data = USBPORT.read(BOX_LENGTH);
	    strcpy(charbuf, command_data.c_str());
	    char *buf = charbuf;

	    cout<<"box target number: "<< target_number<<endl;

	    if (checkEnd(buf,BOX_LENGTH-1)){
	      response_number = target_number;
	      box.data.clear();
	      get_box(buf, box);
	      box_checked[response_number] = check_return_box(agent_box[response_number].box, box);
	      cout<<"Box check: "<<box_checked[response_number]<<endl;
	      boxcheck_msg.agent_number = response_number;
	      boxcheck_msg.check.data = box_checked[response_number];
	      boxcheck_pub.publish(boxcheck_msg);
	      cout<<box<<endl;
	    }
	    else{
	      ROS_INFO_THROTTLE(1,"no end data");
	    }

	  }
	  break;
	case COMMAND_TAKEOFF:
	  {
	    USBPORT.setTimeout(serial::Timeout::max(),150,0,150,0);// adjust timeout
	    char charbuf[256] = {'\0'};
	    string command_data = USBPORT.read(TAKEOFF_LENGTH);
	    strcpy(charbuf, command_data.c_str());
	    char *buf = charbuf;

	    if (checkEnd(buf,TAKEOFF_LENGTH-1)){
	      //only verifies response from the agent
	      response_number = target_number;
	      takeoff_command = get_takeoff_command(buf, response_alg);
	      takeoff_checked[response_number] = check_takeoff(agent_takeoff[response_number].takeoff_command, takeoff_command) && check_alg(alg, response_alg);
	      cout<<"Takeoff check: "<<takeoff_checked[response_number]<<endl;
	      cout<<takeoff_command<<endl;
	    }
	  }
	  break;
	case COMMAND_TARGETE:
	  {
	    ROS_INFO_THROTTLE(1,"Receiving target estimate");
	    USBPORT.setTimeout(serial::Timeout::max(),150,0,150,0);// adjust timeout
	    char charbuf[256] = {'\0'};
	    string command_data = USBPORT.read(TARGETE_LENGTH);
	    strcpy(charbuf, command_data.c_str());
	    char *buf = charbuf;

	    if (checkEnd(buf,TARGETE_LENGTH-1)){
	      get_targetE(buf);
	      if(checkValue(targetE.source.latitude,-180,180) && checkValue(targetE.source.longitude,-180,180) && checkValue(targetE.source.altitude,0,2000)){
		targetE.agent_number = target_number;	
		agent_targetE_pub.publish(targetE);
	      }
	    }
	    break;
	  }
	case COMMAND_REALTARGET:
	  {
	    USBPORT.setTimeout(serial::Timeout::max(),100,0,100,0);// adjust timeout
	    char charbuf[256] = {'\0'};
	    string command_data = USBPORT.read(REALTARGET_LENGTH);
	    strcpy(charbuf, command_data.c_str());
	    char *buf = charbuf;


	    if (checkEnd(buf,REALTARGET_LENGTH-1)){
	      //only verifies response from the agent
	      response_number = target_number;
	      get_realTarget(buf);
	      source_checked[response_number]= check_return_source(agent_source[response_number], realTarget);
	      
	      cout<<"What we think it should be"<<agent_source[response_number].source<<endl;
	      cout<<"source check: "<<source_checked[response_number]<<endl;
	      cout<<realTarget<<endl;
	      sourcecheck_msg.agent_number = response_number;
	      sourcecheck_msg.check.data = source_checked[response_number];
	      sourcecheck_pub.publish(sourcecheck_msg);	      
	    }
	    else{
	      ROS_INFO_THROTTLE(1,"no end data realtarget");
	    }


	  }
	  break;
	default:
	  break;
	}
      }
    }
    ros::spinOnce();
    //loop_rate.sleep();
    ++count;
  }
  return 0;
}
