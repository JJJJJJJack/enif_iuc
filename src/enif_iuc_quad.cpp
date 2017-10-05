#include "enif.h"

//FIXME MPS message
// callback functions

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
  if(mps.gasID.compare("Propane"))
    GAS_ID = GAS_PROPANE;
  else if(mps.gasID.compare("Methane"))
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

int get_target_number(char* buf)
{
  int number = (int)buf[0];
  return number;
}

int get_command_type(char* buf)
{
  int number = (int)buf[1];
  return number;
}

bool get_takeoff_command(char* buf)
{
  if((int)buf[2] == 0)
    return false;
  else
    return true;
}

int get_waypoint_number(char* buf)
{
  return (int)buf[2];
}

void get_waypoint_info(char* buf, enif_iuc::WaypointTask &waypoint_list)
{
  double velocity, damping_distance;
  memcpy(&velocity, buf+3, sizeof(double));
  waypoint_list.velocity = velocity;
  memcpy(&damping_distance, buf+11, sizeof(double));
  waypoint_list.damping_distance = damping_distance;
}

void get_waypoints(int waypoint_number, char* buf, enif_iuc::WaypointTask &waypoint_list)
{
  int byte_number = 0;
  for(int i=0; i<waypoint_number; i++)
    {
      enif_iuc::Waypoint waypoint;
      double latitude, longitude, target_height;
      int staytime;
      // Get latitude
      memcpy(&latitude, buf+19+byte_number, sizeof(double));
      waypoint.latitude = latitude;
      byte_number += sizeof(double);
      // Get longitude
      memcpy(&longitude, buf+19+byte_number, sizeof(double));
      waypoint.longitude = longitude;
      byte_number += sizeof(double);
      // Get waypoint height
      memcpy(&target_height, buf+19+byte_number, sizeof(double));
      waypoint.target_height = target_height;
      byte_number += sizeof(double);
      // Get staytime
      waypoint.staytime = buf[19+byte_number];
      byte_number++;
      waypoint_list.mission_waypoint.push_back(waypoint);
    }
}

void form_mps(char* buf)
{
  buf[0] = AGENT_NUMBER;
  buf[1] = COMMAND_MPS;
  buf[2] = GAS_ID;
  snprintf(buf+3, sizeof(double), "%f", mps.percentLEL);
  snprintf(buf+3+8*1, sizeof(double), "%f", mps.temperature);
  snprintf(buf+3+8*2, sizeof(double), "%f", mps.pressure);
  snprintf(buf+3+8*3, sizeof(double), "%f", mps.humidity);
  snprintf(buf+3+8*4, sizeof(double), "%f", mps.GPS_latitude);
  snprintf(buf+3+8*5, sizeof(double), "%f", mps.GPS_longitude);
  buf[3+8*6] = 0x0A;
  buf[4+8*6] = '\0';
}

void form_GPS(char* buf)
{
  buf[0] = AGENT_NUMBER;
  buf[1] = COMMAND_GPS;
  snprintf(buf+2, sizeof(double), "%f", gps.latitude);
  snprintf(buf+2+sizeof(double), sizeof(double), "%f", gps.longitude);
  buf[18] = gps.status.status;
  buf[19] = gps.altitude;
  snprintf(buf+20, sizeof(double), "%f", height.range);
  buf[28] = 0x0A;
  buf[29] = '\0';
}

void form_state(char* buf)
{
  buf[0] = AGENT_NUMBER;
  buf[1] = COMMAND_STATE;
  buf[2] = state.data;
  buf[3] = 0x0A;
  buf[4] = '\0';
}

void form_battery(char* buf)
{
  buf[0] = AGENT_NUMBER;
  buf[1] = COMMAND_BATTERY;
  snprintf(buf+2, sizeof(double), "%f", battery.voltage);
  buf[10] = 0x0A;
  buf[11] = '\0';
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "enif_iuc_quad");
  ros::NodeHandle n;
  // Receive from Xbee and publish to local ROS
  ros::Publisher  takeoff_pub = n.advertise<std_msgs::Bool>("takeoff_command", 1);
  ros::Publisher  wp_pub      = n.advertise<enif_iuc::WaypointTask>("waypoint_list", 1);
  // Subscribe topics from onboard ROS and transmit it through Xbee
  ros::Subscriber sub_state   = n.subscribe("agentState",1,state_callback);
  ros::Subscriber sub_mps     = n.subscribe("mps_data",1,mps_callback);
  ros::Subscriber sub_GPS     = n.subscribe("mavros/global_position/global",1,GPS_callback);
  ros::Subscriber sub_height  = n.subscribe("ext_height",1,height_callback);
  ros::Subscriber sub_battery = n.subscribe("mavros/battery",1,battery_callback);

  n.getParam("/enif_iuc_quad/AGENT_NUMBER", AGENT_NUMBER);
  cout<<"This is Agent No."<<AGENT_NUMBER<<endl;
  
  ros::Rate loop_rate(100);

  struct timeval tvstart, tvend, timeout;
  gettimeofday(&tvstart,NULL);
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;

  // Start the USB serial port
  FILE *fp;
  fp = fopen("/dev/ttyUSB2", "r+");
  if(fp != NULL)
    cout<<"Success logging PWM at device No."<<fp<<endl;
  else
    cerr<<"Error opening Serial device. Check permission on reading serial port first!"<<endl;
  int fd =  fileno(fp);

  int count = 0, send_count = 0;
  int waypoint_number = 0;
  fd_set fds;

  while (ros::ok())
  {
    char buf[256];
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    int select_result = select(fd+1, &fds, NULL, NULL, &timeout);
    // Timeout every 2 second to make it keep running.
    if(select_result > 0)
      {
	fgets(buf, 256, fp);
	// Get the target number first
	int target_number = get_target_number(buf);
	cout<<"Target number: "<<target_number<<endl;
	if(target_number != AGENT_NUMBER){
	  //Do nothing if target number doesn't match
	  //cout<<buf<<endl;
	  printf("%x %x %x %x\n", buf[0], buf[1], buf[2], buf[3]);
	  cout<<"hello"<<endl;
	}else{
	  // Get command type
	  int command_type = get_command_type(buf);
	  switch(command_type){
	  case COMMAND_WAYPOINT:
	    // Publish waypoint
	    waypoint_list.mission_waypoint.clear();
	    waypoint_number = get_waypoint_number(buf);
	    get_waypoint_info(buf, waypoint_list);
	    get_waypoints(waypoint_number, buf, waypoint_list);
	    wp_pub.publish(waypoint_list);
	    cout<<"Receiving wp message:  "<<buf<<endl;
	    break;
	  case COMMAND_TAKEOFF:
	    takeoff_command.data = get_takeoff_command(buf);
	    takeoff_pub.publish(takeoff_command);
	    cout<<"Receiving takeoff command:  "<<buf<<endl;
	    break;
	  default:
	    break;
	  }
	}
      }
    // Send GPS mps state and battery data every 1 sec
    if(count%25 == 0)
      {
	char send_buf[256];
	switch(send_count){
	case 0:
	  if(NEW_MPS){
	    form_mps(send_buf);
	    fputs(send_buf, fp);
	    NEW_MPS = false;
	  }
	  break;
	case 1:
	  if(NEW_GPS){
	    form_GPS(send_buf);
	    fputs(send_buf, fp);
	    NEW_GPS = false;
	  }
	  break;
	case 2:
	  if(NEW_STATE){
	    form_state(send_buf);
	    fputs(send_buf, fp);
	    NEW_STATE = false;
	  }
	  break;
	case 3:
	  if(NEW_BATTERY){
	    form_battery(send_buf);
	    fputs(send_buf, fp);
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
  
  fclose(fp);
  return 0;
}
  
