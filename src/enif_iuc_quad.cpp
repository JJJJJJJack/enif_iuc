#include "enif.h"

#include "enif_iuc/AgentTakeoff.h"
#include "enif_iuc/AgentWaypointTask.h"
#include "enif_iuc/AgentGlobalPosition.h"
#include "enif_iuc/AgentMPS.h"
#include "enif_iuc/AgentState.h"
#include "enif_iuc/AgentHeight.h"
#include "enif_iuc/AgentBatteryState.h"
#include "enif_iuc/AgentBox.h"
#include "enif_iuc/AgentHome.h"
#include "enif_iuc/AgentLocal.h"

bool NEW_STATE = false, NEW_MPS = false, NEW_GPS = false, NEW_HEIGHT = false, NEW_BATTERY = false, NEW_HOME = false, NEW_LOCAL = false, NEW_TARGETE=false, NEW_REALTARGET=false, NEW_BOX=false;

using namespace std;

void state_callback(const std_msgs::UInt8 &new_message)
{
  state = new_message;
  NEW_STATE = true;
}

void home_callback(const mavros_msgs::HomePosition &new_message)
{
  home = new_message;
  NEW_HOME = true;
}

void targetEGPS_callback(const geographic_msgs::GeoPoint &new_message)
{
  targetE = new_message;
  NEW_TARGETE = true;
}

void local_callback(const nav_msgs::Odometry &new_message)
{
  local = new_message;
  NEW_LOCAL = true;
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
  NEW_GPS = true;
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
  FloatToChar(buf+4+8, height.range);
  FloatToChar(buf+4+12, mps.humidity);
  DoubleToChar(buf+4+16, gps.latitude);
  DoubleToChar(buf+4+24, gps.longitude);
  DoubleToChar(buf+4+32, gps.altitude);
  buf[4+40] = 0x0A;
  // Clear the percentLEL to make sure we don't pub wrong data when we get new GPS
  clearmps();
}

void form_targetE(char* buf)
{
  buf[1] = IntToChar(AGENT_NUMBER);
  buf[2] = IntToChar(COMMAND_TARGETE);
  DoubleToChar(buf+3, targetE.latitude);
  DoubleToChar(buf+3+8, targetE.longitude);
  DoubleToChar(buf+3+8+8, targetE.altitude);
  buf[3+8+8+8] = 0x0A;
}

void form_local(char* buf)
{
  buf[1] = IntToChar(AGENT_NUMBER); // 1 byte
  buf[2] = IntToChar(COMMAND_LOCAL);
  DoubleToChar(buf+3, local.pose.pose.position.x);
  DoubleToChar(buf+11, local.pose.pose.position.y);
  DoubleToChar(buf+19, local.pose.pose.position.z);
  DoubleToChar(buf+27, local.pose.pose.orientation.x);
  DoubleToChar(buf+35, local.pose.pose.orientation.y);
  DoubleToChar(buf+43, local.pose.pose.orientation.z);
  DoubleToChar(buf+51, local.pose.pose.orientation.w);
  buf[51+8] = 0x0A;
}

void form_home(char* buf)
{
  buf[1] = IntToChar(AGENT_NUMBER);
  buf[2] = IntToChar(COMMAND_HOME);
  DoubleToChar(buf+3, home.geo.latitude);
  DoubleToChar(buf+11, home.geo.longitude);
  DoubleToChar(buf+19, home.geo.altitude);
  buf[19+8] = 0x0A;
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
  ros::Publisher  mps_pub     = n.advertise<enif_iuc::AgentMPS>("agent_mps_data", 1);

  
  ros::Publisher  home_pub    = n.advertise<enif_iuc::AgentHome>("agent_home_data", 1);
  ros::Publisher  local_pub   = n.advertise<enif_iuc::AgentLocal>("agent_local_data", 1);
  ros::Publisher  realTarget_pub  = n.advertise<enif_iuc::AgentSource>("agent_source_data", 1);
  
  // Subscribe topics from onboard ROS and transmit it through Xbee
  ros::Subscriber sub_state   = n.subscribe("agentState",1,state_callback);
  ros::Subscriber sub_mps     = n.subscribe("mps_data",1,mps_callback);
  ros::Subscriber sub_GPS     = n.subscribe("mavros/global_position/global",1,GPS_callback);
  ros::Subscriber sub_height  = n.subscribe("mavros/distance_sensor/lidarlite_pub",1,height_callback);
  ros::Subscriber sub_battery = n.subscribe("mavros/battery",1,battery_callback);
  
  ros::Subscriber sub_targetE    = n.subscribe("/pf/targetGPS_",1, targetEGPS_callback);

  ros::Subscriber sub_home    = n.subscribe("/mavros/home_position/home",1,home_callback);
  ros::Subscriber sub_local   = n.subscribe("/mavros/global_position/local",1,local_callback);


  n.getParam("/enif_iuc_quad/AGENT_NUMBER", AGENT_NUMBER);
  cout<<"This is Agent No."<<AGENT_NUMBER<<endl;
  
  n.getParam("/enif_iuc_quad/sendLocal", sendLocal);
  n.getParam("/enif_iuc_quad/sendHome", sendHome);
  n.getParam("/enif_iuc_quad/sendBat", sendBat);
  n.getParam("/enif_iuc_quad/sendTargetE", sendTargetE);
  n.getParam("/enif_iuc_quad/sendRealTarget", sendRealTarget);
  cout<<"sendLocal: "<<sendLocal<<endl;
  cout<<"sendHome: "<<sendHome<<endl;
  cout<<"sendBat: "<<sendBat<<endl;
  
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
    char charbuf[256] = {'\0'};
    string data = USBPORT.read(256+1);
    strcpy(charbuf, data.c_str());
    char *buf = charbuf;
    int command_type = get_command_type(buf);
    int c = 0;
    while(buf[0] != '\0' && (command_type==COMMAND_MPS || command_type==COMMAND_HOME || command_type==COMMAND_LOCAL || command_type==COMMAND_BOX || command_type==COMMAND_TAKEOFF || command_type==COMMAND_WAYPOINT) || command_type==COMMAND_REALTARGET){
      command_type = get_command_type(buf);
      //      cout<<strlen(buf)<<endl;
      //RnnnnnnOS_INFO_THROTTLE(1,"%d",strlen(buf));
      //printf("%x\n", buf[0]);
    // Get the target number first
    int target_number = get_target_number(buf);

    if(target_number != AGENT_NUMBER){
      if(target_number > 0){
	// Get command type

	bool checksum_result = false;
	enif_iuc::AgentMPS agent_mps;	
	enif_iuc::AgentLocal agent_local;
	
	sensor_msgs::NavSatFix my_gps = gps;
	mps_driver::MPS my_mps = mps;
	
	mavros_msgs::HomePosition my_home;
	nav_msgs::Odometry my_local = local;

	switch(command_type){
	case COMMAND_MPS:
	  //form mps and publish
	  //cout<<"Receiving mps quad info ";
	  ROS_INFO_THROTTLE(1,"Receiving mps quad info from Agent %d", target_number);
	  checksum_result = checksum(buf);
	  get_other_mps(buf);
	  buf = buf + 45;
	  agent_mps.agent_number = target_number;
	  agent_mps.mps = mps_other;
	  if(check_MPS(mps_other)){
	    mps_pub.publish(agent_mps);
	  }
	  if(NEW_MPS){
	    mps = my_mps; gps = my_gps;
	  }
	  //cout<<"from Agent."<<target_number<<endl;
	  break;
	case COMMAND_REALTARGET:
	  if (target_number==100)
	    {
	      //get source and publish
	      ROS_INFO_THROTTLE(1,"Receiving source point");
	      //cout<<"Receiving home quad info ";
	      checksum_result = checksum(buf);
	      get_realTarget(buf);

	      if(checkValue(realTarget.source.latitude,-180,180) && checkValue(realTarget.source.longitude,-180,180) && checkValue(realTarget.source.altitude,0,2000)){
		//publish the source position
	
		//set the home location to be the same as the source location
		agent_home.home.header.stamp = ros::Time::now();		
		agent_home.home.geo.latitude = realTarget.source.latitude;
		agent_home.home.geo.longitude = realTarget.source.longitude;
		agent_home.home.geo.altitude = realTarget.source.altitude;
		NEW_REALTARGET = true;
	      }
	      int tempbuf_size = 42;
	      char tempbuf[tempbuf_size];
	      cut_buf(buf, tempbuf, tempbuf_size);
	      buf+=tempbuf_size;
	      tempbuf[1]=IntToChar(AGENT_NUMBER);
	      string send_data(tempbuf);
	      USBPORT.write(send_data);
	      //std::cout<<"sendingData"<<std::endl;
	    }
	  break;	  
	default:
	  break;
	}
	if(command_type==COMMAND_WAYPOINT || command_type==COMMAND_TAKEOFF || command_type==COMMAND_BOX)
	  break;
      }
    }else{
      // Get command type
      cout<<"Receiving command: ";
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

	int tempbuf_size = get_waypointlist_buf_size(waypoint_number)+1;
	char tempbuf[tempbuf_size];
	cut_buf(buf, tempbuf, tempbuf_size);
	buf+=tempbuf_size;
	string send_data(tempbuf);
	USBPORT.write(send_data);
	buf += tempbuf_size;
	break;
      }
      case COMMAND_BOX:{
	cout<< " Box"<<endl;
	box.data.clear();
	get_box(buf, box);
	checksum_result = checksum(buf);
	cout<<box<<endl;
	NEW_BOX = true;
	int tempbuf_size = 50;
	char tempbuf[tempbuf_size];
	cut_buf(buf, tempbuf, tempbuf_size);
	buf+=tempbuf_size;
	string send_data(tempbuf);
	USBPORT.write(send_data);
	break;
      }
      case COMMAND_TAKEOFF:{
	std_msgs::Bool cmd = get_takeoff_command(buf, alg);

	switch (alg.data) {
	case 1 :	  
	  n.setParam("/runAlg", "lawnMower");
	  break;
	case 2 :
	  n.setParam("/runAlg", "PSO");
	  break;
	case 3 :
	  n.setParam("/runAlg", "PF");
	  break;
	default :
	  n.setParam("/runAlg", "lawnMower");
	}

	takeoff_command.data = cmd.data;
	if(takeoff_command.data == true)
	  cout<< " Takeoff"<<endl;
	else
	  cout<< " Land"<<endl;
	checksum_result = checksum(buf);
	int tempbuf_size = 5;
	char tempbuf[tempbuf_size];
	cut_buf(buf, tempbuf, tempbuf_size);
	buf+=tempbuf_size;
	string send_data(tempbuf);
	USBPORT.write(send_data);
	break;
      }
      default:
	break;
      }
      //if(checksum_result)
      {
	if(command_type == COMMAND_WAYPOINT)
	  wp_pub.publish(waypoint_list);
	takeoff_pub.publish(takeoff_command);
	//if(command_type == COMMAND_BOX)
	//  box_pub.publish(box);
      }
      break;
    }
    c++;
    if (c>50)
      {
	break;
      }
    }
    if (NEW_REALTARGET){
      realTarget_pub.publish(realTarget);
      home_pub.publish(agent_home);
    }
    if (NEW_BOX){
      box_pub.publish(box);
    }
    

    // Send GPS mps state and battery data every 1 sec
    if(count%2 == 0)
      {
	char send_buf[256] = {'\0'};
	switch(send_count){
	case 0:
	  if(NEW_GPS){
	    form_mps(send_buf);
	    form_checksum(send_buf);
	    string send_data(send_buf);
	    USBPORT.write(send_data);
	    NEW_GPS = false;
	  }
	  break;
	case 1:
	  if(NEW_STATE){
	    form_state(send_buf);
	    form_checksum(send_buf);
	    string send_data(send_buf);
	    USBPORT.write(send_data);
	    NEW_STATE = false;
	  }
	  break;
	case 2:
	  if(NEW_TARGETE && sendTargetE){	    
	    form_targetE(send_buf);
	    form_checksum(send_buf);	    
	    string send_data(send_buf);	    
	    USBPORT.write(send_data);
	    NEW_TARGETE = false;	    
	  }
	  break;
	default:
	  break;
	}
	if(send_count <= 2) send_count++;
	else send_count = 0;       
      }    
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;

  }
  return 0;
}
  
