#include "enif.h"

#include "enif_iuc/AgentTakeoff.h"
#include "enif_iuc/AgentWaypointTask.h"
#include "enif_iuc/AgentGlobalPosition.h"
#include "enif_iuc/AgentMPS.h"
#include "enif_iuc/AgentState.h"


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

void get_state(char* buf)
{
  state.data = (int)buf[2];
}

void get_mps(char* buf)
{
  GAS_ID = buf[2];
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
  double percentLEL, temperature, pressure, humidity, GPS_latitude, GPS_longitude;
  memcpy(&percentLEL, buf+3, sizeof(double));
  mps.percentLEL = percentLEL;
  memcpy(&temperature, buf+3, sizeof(double));
  mps.temperature = temperature;
  memcpy(&pressure, buf+3, sizeof(double));
  mps.pressure = pressure;
  memcpy(&humidity, buf+3, sizeof(double));
  mps.humidity = humidity;
  memcpy(&GPS_latitude, buf+3, sizeof(double));
  mps.GPS_latitude = GPS_latitude;
  memcpy(&GPS_longitude, buf+3, sizeof(double));
  mps.GPS_longitude = GPS_longitude;
}

void get_GPS(char* buf)
{
  double latitude, longitude;
  int health = 0;
  memcpy(&latitude, buf+2, sizeof(double));
  gps.latitude = latitude;
  memcpy(&longitude, buf+2+sizeof(double), sizeof(double));
  gps.longitude = longitude;
  gps.health = buf[18];
}

void form_takeoff(char* buf, int agent_number, bool takeoff)
{
  buf[0] = agent_number;
  buf[1] = COMMAND_TAKEOFF;
  buf[2] = takeoff;
  buf[3] = 0x0A;
}

void form_waypoint_info(char* buf, int agent_number, int waypoint_number, float velocity)
{
  buf[0] = agent_number;
  buf[1] = COMMAND_WAYPOINT;
  buf[2] = waypoint_number;
  snprintf(buf+3, sizeof(double), "%f", velocity);
}

void form_waypoints(char* buf, int waypoint_number, enif_iuc::WaypointTask &waypoint_list)
{
  int byte_number = 0;
  for(int i=0; i<waypoint_number; i++)
    {
      snprintf(buf+11+byte_number, sizeof(double), "%f", waypoint_list.mission_waypoint[i].latitude);
      byte_number += sizeof(double);
      snprintf(buf+11+byte_number, sizeof(double), "%f", waypoint_list.mission_waypoint[i].longitude);
      byte_number += sizeof(double);
    }
  buf[11+byte_number] = 0x0A;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "enif_iuc_ground");
  ros::NodeHandle n;

  ros::Publisher  state_pub    = n.advertise<enif_iuc::AgentState>("agentState", 1);
  ros::Publisher  mps_pub      = n.advertise<enif_iuc::AgentMPS>("mps_data", 1);
  ros::Publisher  GPS_pub      = n.advertise<enif_iuc::AgentGlobalPosition>("global_position", 1);
  ros::Subscriber sub_takeoff  = n.subscribe("takeoff_command",1,takeoff_callback);
  ros::Subscriber sub_wp       = n.subscribe("waypoint_list",1,wp_callback);
  
  ros::Rate loop_rate(100);

  struct timeval tvstart, tvend, timeout;
  gettimeofday(&tvstart,NULL);
  timeout.tv_sec = 2;
  timeout.tv_usec = 0;

  // Start the USB serial port
  FILE *fp;
  fp = fopen("/dev/ttyUSB0", "r+");
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
    //signal_pub.publish(data);
    //goal_pub.publish(goal);
    char buf[256];
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    int select_result = select(fd+1, &fds, NULL, NULL, &timeout);
    // Timeout every 2 seconds to make it keep running.
    if(select_result > 0)
      {
	fgets(buf, 256, fp);
	// Get the target number first
	int target_number = get_target_number(buf);
	cout<<"Target number: "<<target_number<<endl;
	if(false){
	  //Do nothing because GS need to subs to all other topics
	  cout<<buf<<endl;
	  cout<<"hello"<<endl;
	}else{
	  // Get command type
	  int command_type = get_command_type(buf);
	  enif_iuc::AgentGlobalPosition agent_gps;
	  enif_iuc::AgentMPS agent_mps;
	  enif_iuc::AgentState agent_state;
	  switch(command_type){
	  case COMMAND_GPS:
	    //form GPS and publish
	    get_GPS(buf);
	    agent_gps.agent_number = target_number;
	    agent_gps.gps = gps;
	    GPS_pub.publish(agent_gps);
	    break;
	  case COMMAND_MPS:
	    //form mps and publish
	    get_mps(buf);
	    agent_mps.agent_number = target_number;
	    agent_mps.mps = mps;
	    mps_pub.publish(agent_mps);
	    break;
	  case COMMAND_STATE:
	    //form state and publish
	    get_state(buf);
	    agent_state.agent_number = target_number;
	    agent_state.state = state;
	    state_pub.publish(agent_state);
	    break;
	  default:
	    break;
	  }
	}
      }
    if(count%10 == 0)
      {
	char send_buf[256];
	if(NEW_TAKEOFF)
	  {
	    form_takeoff(send_buf, agent_takeoff.agent_number, agent_takeoff.takeoff_command);
	    NEW_TAKEOFF = false;
	    fputs(send_buf, fp);
	  }
	else if(NEW_WP)
	  {
	    form_waypoint_info(send_buf, agent_wp.agent_number, agent_wp.waypoint_list.mission_waypoint.size(), agent_wp.waypoint_list.idle_velocity);
	    form_waypoints(send_buf, agent_wp.waypoint_list.mission_waypoint.size(), agent_wp.waypoint_list);
	    NEW_WP = false;
	    fputs(send_buf, fp);
	  }
      }

    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
