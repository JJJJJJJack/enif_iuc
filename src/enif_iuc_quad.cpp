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

bool ERROR_CA = false, ERROR_LIDAR = false, ERROR_ALTITUDE = false, ERROR_GPS = false, ERROR_MPS = false;
double CA_update_sec, Lidar_update_sec, GPS_update_sec, MPS_update_sec;
/*------------Error info format---------------
The error info is combined with agentState in the state command

 -      -     -    -    -   |   - - -
GPS   Lidar  CA   Alt  MPS  | agentState

---------------------------------------------*/

serial::Serial USBPORT("/dev/xbee", 57600, serial::Timeout::simpleTimeout(1000));

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

void targetEGPS_callback(const enif_iuc::AgentSource &new_message)
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
  ERROR_MPS = false;
  MPS_update_sec = ros::Time::now().toSec();
}

void vel_callback(const geometry_msgs::TwistStamped& msg)
{
  vel = msg;
}
 
void GPS_callback(const sensor_msgs::NavSatFix &new_message)
{
  gps = new_message;
  NEW_GPS = true;
  ERROR_GPS = false;
  GPS_update_sec = ros::Time::now().toSec();
  if(gps.altitude > 0)
    ERROR_ALTITUDE = false;
}

void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &new_message)
{
  ERROR_LIDAR = false;
  Lidar_update_sec = ros::Time::now().toSec();
}

void CA_callback(const geometry_msgs::Twist::ConstPtr &new_message)
{
  ERROR_CA = false;
  CA_update_sec = ros::Time::now().toSec();
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
  buf[2] = IntToChar(AGENT_NUMBER);
  buf[3] = IntToChar(COMMAND_MPS);
  FloatToChar(buf+4, mps.percentLEL);
  FloatToChar(buf+8, height.range);
  DoubleToChar(buf+12, gps.latitude);
  DoubleToChar(buf+20, gps.longitude);

  FloatToChar(buf+28, vel.twist.linear.x);
  FloatToChar(buf+32, vel.twist.linear.y);
  FloatToChar(buf+36, vel.twist.linear.z);  

  //DoubleToChar(buf+28, gps.altitude);
  
  //FloatToChar(buf+36, vel.twist.linear.x);
  //FloatToChar(buf+40, vel.twist.linear.y);
  //FloatToChar(buf+44, vel.twist.linear.z);  
  
  buf[40] = 0x0A;
  // Clear the percentLEL to make sure we don't pub wrong data when we get new GPS

  clearmps();
}

void form_targetE(char* buf)
{
  buf[2] = IntToChar(AGENT_NUMBER);
  buf[3] = IntToChar(COMMAND_TARGETE);
  DoubleToChar(buf+4, targetE.source.latitude);
  DoubleToChar(buf+12, targetE.source.longitude);
  DoubleToChar(buf+20, targetE.source.altitude);
  
  FloatToChar(buf+28, targetE.angle);
  FloatToChar(buf+32, targetE.wind_speed);
  FloatToChar(buf+36, targetE.diff_y);
  FloatToChar(buf+40, targetE.diff_z);
  FloatToChar(buf+44, targetE.release_rate);  
  
  buf[48] = 0x0A;
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
  //Adding error info into state
  int error_info = ERROR_GPS<<7 | ERROR_LIDAR<<6 | ERROR_CA<<5 | ERROR_ALTITUDE<<4 | ERROR_MPS<<3 | state.data;
  buf[2] = IntToChar(AGENT_NUMBER);
  buf[3] = IntToChar(COMMAND_STATE);
  buf[4] = IntToChar(error_info);
  buf[5] = 0x0A;
}

void form_battery(char* buf)
{
  buf[1] = IntToChar(AGENT_NUMBER);
  buf[2] = IntToChar(COMMAND_BATTERY);
  FloatToChar(buf+3, battery.voltage);
  buf[7] = 0x0A;
}

void transmitData_MPS(const ros::TimerEvent& event)
{
  char send_buf[256] = {'\0'};
  form_start(send_buf);
  if(NEW_GPS){
    if(!NEW_MPS){
      mps.percentLEL = -1;
    }
    form_mps(send_buf);
    //form_checksum(send_buf);
    string send_data(send_buf);	    
    USBPORT.write(send_data);
    NEW_MPS=false;
    NEW_GPS=false;
  }
}

void transmitData_targetE(const ros::TimerEvent& event)
{
  char send_buf[256] = {'\0'};
  form_start(send_buf);
  if(NEW_TARGETE && sendTargetE){
    form_targetE(send_buf);
    //form_checksum(send_buf);	    
    string send_data(send_buf);	    
    USBPORT.write(send_data);
    NEW_TARGETE = false;	    
  } 
}

void transmitData_state(const ros::TimerEvent& event)
{
  char send_buf[256] = {'\0'};
  form_start(send_buf);
  if(NEW_STATE){
    form_state(send_buf);
    //form_checksum(send_buf);
    string send_data(send_buf);
    USBPORT.write(send_data);
    NEW_STATE = false;
  }
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
  ros::Publisher  mle_pub     = n.advertise<enif_iuc::AgentSource>("agent_mle_data", 1);
  
  ros::Publisher  home_pub    = n.advertise<enif_iuc::AgentHome>("agent_home_data", 1);
  ros::Publisher  local_pub   = n.advertise<enif_iuc::AgentLocal>("agent_local_data", 1);
  ros::Publisher  realTarget_pub  = n.advertise<enif_iuc::AgentSource>("agent_source_data", 1);
  // Subscribe topics from onboard ROS and transmit it through Xbee
  ros::Subscriber sub_state   = n.subscribe("agentState",1,state_callback);
  ros::Subscriber sub_mps     = n.subscribe("mps_data",1,mps_callback);
  ros::Subscriber sub_GPS     = n.subscribe("mavros/global_position/global" ,1, GPS_callback);  
  ros::Subscriber sub_vel     = n.subscribe("mavros/local_position/velocity",1, vel_callback);
  
  ros::Subscriber sub_height  = n.subscribe("mavros/distance_sensor/lidarlite_pub",1,height_callback);
  ros::Subscriber sub_battery = n.subscribe("mavros/battery",1,battery_callback);
  
  ros::Subscriber sub_targetE    = n.subscribe("/pf/targetGPS_",1, targetEGPS_callback);

  ros::Subscriber sub_home    = n.subscribe("/mavros/home_position/home",1,home_callback);
  ros::Subscriber sub_local   = n.subscribe("/mavros/global_position/local",1,local_callback);
  // Following topics are only subscribed for debugging
  ros::Subscriber sub_lidar   = n.subscribe("/scan",1,lidar_callback);
  ros::Subscriber sub_CA      = n.subscribe("/cmd_vel",1,CA_callback);

  n.param<double>("/enif_iuc_quad/transmitRate", transmitRate, .35);
  
  ros::Timer transmit_timer_MPS     = n.createTimer(ros::Duration(transmitRate), transmitData_MPS);  
  ros::Timer transmit_timer_targetE = n.createTimer(ros::Duration(2), transmitData_targetE);
  ros::Timer transmit_timer_state   = n.createTimer(ros::Duration(10), transmitData_state);
   
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
  cout<<"sendTargetE: "<<sendTargetE<<endl;
  cout<<"sendRealTarget: "<<sendRealTarget<<endl;
  
  ros::Rate loop_rate(1000);

  // Set timeout for XBEE reading
  struct timeval tvstart, tvend, timeout;
  gettimeofday(&tvstart,NULL);
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  // Initialize error info update time
  double current_time_sec = ros::Time::now().toSec();
  GPS_update_sec = current_time_sec;
  Lidar_update_sec = current_time_sec;
  MPS_update_sec = current_time_sec;
  CA_update_sec = current_time_sec;

  // Start the USB serial port
  //n.getParam("/enif_iuc_quad/USB", USB);
  
  
  if(USBPORT.isOpen())
    cout<<"Wireless UART port opened"<<endl;
  else
    cout<<"Error opening Serial device. Check permission on reading serial port first!"<<endl;

  int count = 0, send_count = 0;
  int waypoint_number = 0;

  while (ros::ok())
    {
      data = USBPORT.read(1);
      // check for start
      if (!data.compare("<")){
	data = USBPORT.read(1);
	if (!data.compare("<")){

	  // process data	  
	  data = USBPORT.read(2);	  
	  char Tcharbuf[4] = {'\0'};
	  char *Tbuf = Tcharbuf;
	  strcpy(Tcharbuf, data.c_str());

	  int command_type = get_command_type(Tbuf);
	  int target_number = get_target_number(Tbuf);

	  switch(command_type){
	  case COMMAND_MPS:
	    {
	      USBPORT.setTimeout(serial::Timeout::max(),100,0,100,0);// adjust timeout
	      enif_iuc::AgentMPS agent_mps;
	      char charbuf[256] = {'\0'};
	      string command_data = USBPORT.read(MPS_LENGTH);
	      strcpy(charbuf, command_data.c_str());
	      char *buf = charbuf;

	      //form mps and publish
	      ROS_INFO_THROTTLE(1,"Receiving mps quad info from Agent %d", target_number);
	    
	      if (checkEnd(buf, MPS_LENGTH-1)){
		get_other_mps(buf);
		agent_mps.agent_number = target_number;
		agent_mps.mps = mps_other;
		if(check_MPS(mps_other)){
		  mps_pub.publish(agent_mps);		
		}
	      }
	      else{
		ROS_INFO_THROTTLE(1,"no end data");
	      }
	    }
	    break;
	  case COMMAND_REALTARGET:
	    {
	      if (target_number==100)
		{
		  //get source and publish
		  ROS_INFO_THROTTLE(1,"Receiving source point");
		
		  USBPORT.setTimeout(serial::Timeout::max(),100,0,100,0);// adjust timeout
		  char charbuf[256] = {'\0'};
		  string command_data = USBPORT.read(REALTARGET_LENGTH);
		  strcpy(charbuf, command_data.c_str());
		  char *buf = charbuf;

		  if (checkEnd(buf,REALTARGET_LENGTH-1)){
		    get_realTarget(buf);
		    if(checkValue(realTarget.source.latitude,-180,180) && checkValue(realTarget.source.longitude,-180,180) && checkValue(targetE_other.source.altitude,0,2000)){
		      //publish the source position       
		      realTarget_pub.publish(realTarget);
		      //home_pub.publish(agent_home);
		
		      char send_buf[256] = {'\0'};
		      char start_buf[100] = {'\0'};
		      form_start(start_buf);
		      start_buf[2] = IntToChar(AGENT_NUMBER);
		      start_buf[3] = IntToChar(COMMAND_REALTARGET);
	
		      strcpy(send_buf, buf);
		  
		      string start_data(start_buf);
		      string send_data(send_buf);

		      send_data.insert(0,start_data); // insert start << at beginning
		      USBPORT.write(send_data);
		    }
		  }		
		}
	    }
	    break;
	  case COMMAND_TARGETE:
	    {
	      //get source and publish
	      ROS_INFO_THROTTLE(1,"Receiving target estimate");
	      USBPORT.setTimeout(serial::Timeout::max(),150,0,150,0);// adjust timeout
	      char charbuf[256] = {'\0'};
	      string command_data = USBPORT.read(TARGETE_LENGTH);
	      strcpy(charbuf, command_data.c_str());
	      char *buf = charbuf;

	      if (checkEnd(buf,TARGETE_LENGTH-1)){
		get_targetE_other(buf);
		if(checkValue(targetE_other.source.latitude,-180,180) && checkValue(targetE_other.source.longitude,-180,180) && checkValue(targetE_other.source.altitude,0,2000)){ 
		  targetE_other.agent_number = target_number;
		  mle_pub.publish(targetE_other);
		}	      
	      }
	    }	    
	    break;
	    
	  case COMMAND_WAYPOINT:
	    {
	      if (target_number==AGENT_NUMBER)
		{

		  // Publish waypoint
		  cout<< " Waypoint"<<endl;
		  USBPORT.setTimeout(serial::Timeout::max(),1000,0,1000,0);// adjust timeout

		  char Wcharbuf[256] = {'\0'};
		  string Wcommand_data = USBPORT.read(1); // get number of waypoints
		  strcpy(Wcharbuf, Wcommand_data.c_str());
		  char *Wbuf = Wcharbuf;
		  waypoint_number = get_waypoint_number(Wbuf);

		  int numBytes = get_waypointlist_buf_size(waypoint_number)+1;

		  cout<<"waypoint_number: "<<waypoint_number<<endl;
		  cout<<"numBytes: "<<numBytes<<endl;
		  
		  char charbuf[256] = {'\0'};
		  string command_data = USBPORT.read(numBytes); 
		  strcpy(charbuf, command_data.c_str());
		  char *buf = charbuf;

		  if (checkEnd(buf,numBytes-1)){
		    waypoint_list.mission_waypoint.clear();
		    get_waypoint_info(buf, waypoint_list);
		    get_waypoints(waypoint_number, buf, waypoint_list);
		    cout<<waypoint_list<<endl;
		    wp_pub.publish(waypoint_list);
	      
		    char send_buf[256] = {'\0'};
		    char start_buf[100] = {'\0'};
		    form_start(start_buf);
		    start_buf[2] = IntToChar(target_number);
		    start_buf[3] = IntToChar(COMMAND_WAYPOINT);
		    start_buf[4] = IntToChar(waypoint_number);
		  
		    strcpy(send_buf, buf);
		  
		    string start_data(start_buf);
		    string send_data(send_buf);

		    send_data.insert(0,start_data); // insert start << at beginning
		    USBPORT.write(send_data);
		  }
		}
	    }
	  break;

	  case COMMAND_BOX:{
	    if (target_number==AGENT_NUMBER)
	      {
		cout<< " Box"<<endl;
		USBPORT.setTimeout(serial::Timeout::max(),150,0,150,0);// adjust timeout
		char charbuf[256] = {'\0'};
		string command_data = USBPORT.read(BOX_LENGTH);
		strcpy(charbuf, command_data.c_str());
		char *buf = charbuf;

		if (checkEnd(buf,BOX_LENGTH-1)){
		  box.data.clear();
		  NEW_BOX = get_box(buf, box);
		  box_pub.publish(box);
	      
		  char send_buf[256] = {'\0'};
		  char start_buf[100] = {'\0'};
		  form_start(start_buf);
		  start_buf[2] = IntToChar(target_number);
		  start_buf[3] = IntToChar(COMMAND_BOX);
		  
		  strcpy(send_buf, buf);
		  //send_buf[BOX_LENGTH] = 0x0A;
		  
		  string start_data(start_buf);
		  string send_data(send_buf);

		  send_data.insert(0,start_data); // insert start << at beginning
		  USBPORT.write(send_data);
		}
		else{
		  ROS_INFO_THROTTLE(1,"no end data");
		}
	    
	      }
	  }
	    break;
	    
	  case COMMAND_TAKEOFF:{
	    if (target_number==AGENT_NUMBER)
	      {

		USBPORT.setTimeout(serial::Timeout::max(),150,0,150,0);// adjust timeout
		char charbuf[256] = {'\0'};
		string command_data = USBPORT.read(TAKEOFF_LENGTH);
		strcpy(charbuf, command_data.c_str());
		char *buf = charbuf;

		if (checkEnd(buf,TAKEOFF_LENGTH-1)){
		  std_msgs::Bool cmd = get_takeoff_command(buf, alg);

		  switch (alg.data) {
		  case ALG_WAYPOINT :
		    n.setParam("/runAlg", "Waypoint");
		    break;
		  case ALG_LAWNMOWER :	  
		    n.setParam("/runAlg", "lawnMower");
		    break;
		  case ALG_PSO :
		    n.setParam("/runAlg", "PSO");
		    break;
		  case ALG_PF :
		    n.setParam("/runAlg", "PF");
		    break;
		  case ALG_INFO :
		    n.setParam("/runAlg", "info");
		    break;
		  default :
		    n.setParam("/runAlg", "lawnMower");
		  }

		  takeoff_command.data = cmd.data;
		  if(takeoff_command.data == true)
		    cout<< " Takeoff"<<endl;
		  else
		    cout<< " Land"<<endl;	    
		  takeoff_pub.publish(takeoff_command);

		  char send_buf[256] = {'\0'};
		  char start_buf[100] = {'\0'};
		  form_start(start_buf);
		  start_buf[2] = IntToChar(target_number);
		  start_buf[3] = IntToChar(COMMAND_TAKEOFF);

		  
		  strcpy(send_buf, buf);
		  
		  string start_data(start_buf);
		  string send_data(send_buf);

		  send_data.insert(0,start_data); // insert start << at beginning
		  USBPORT.write(send_data);	      
		}
	      }
	    break;
	  }

	  default:
	    break;
	  }
	}
      }
    
    //Check Error messages
    current_time_sec = ros::Time::now().toSec();
    double Lidar_interval = current_time_sec - Lidar_update_sec;
    double CA_interval    = current_time_sec - CA_update_sec;
    double GPS_interval   = current_time_sec - GPS_update_sec;
    double MPS_interval   = current_time_sec - MPS_update_sec;
    if(Lidar_interval > 0.5)
      ERROR_LIDAR = true;
    if(CA_interval > 0.5)
      ERROR_CA = true;
    if(GPS_interval > 0.5)
      ERROR_GPS = true;
    if(gps.altitude < 0)
      ERROR_ALTITUDE = true;
    if(MPS_interval > 0.5)
      ERROR_MPS = true;
    
    ros::spinOnce();
    //loop_rate.sleep();
    ++count;

  }
  return 0;
}
  
