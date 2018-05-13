#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <time.h>
#include <ctime>
#include <unistd.h>
#include <typeinfo>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include "serial/serial.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Float64MultiArray.h"
#include "enif_iuc/Waypoint.h"
#include "enif_iuc/WaypointTask.h"
#include "mps_driver/MPS.h"
#include <string>

using namespace std;

#define COMMAND_TAKEOFF  1
#define COMMAND_WAYPOINT 2
#define COMMAND_GPS      3
#define COMMAND_MPS      4
#define COMMAND_STATE    5
#define COMMAND_BATTERY  6
#define COMMAND_BOX      7

#define GAS_NONE    0
#define GAS_PROPANE 1
#define GAS_METHANE 2

std::string USB;
int AGENT_NUMBER = 0;
int GS_ID = 0;
int GAS_ID = 0;

std_msgs::UInt8 state;
sensor_msgs::NavSatFix gps;
mps_driver::MPS mps;
sensor_msgs::Range height;
sensor_msgs::BatteryState battery;

std_msgs::Bool takeoff_command;
enif_iuc::WaypointTask waypoint_list;
std_msgs::Float64MultiArray box;

int CharToInt(char a)
{
  return (int)(a - '0');
}

char IntToChar(int a)
{
  return (char)(a + '0');
}

void DoubleToChar(char* buf, double number)
{
  unsigned char *chptr;
  chptr = (unsigned char *) &number;
  for(int i = 0; i<sizeof(double); i++){
    buf[i] = *chptr + '0';
    chptr++;
  }
}

void FloatToChar(char* buf, float number)
{
  unsigned char *chptr;
  chptr = (unsigned char *) &number;
  for(int i = 0; i<sizeof(float); i++){
    buf[i] = *chptr + '0';
    chptr++;
  }
}

void CharToDouble(char* buf, double &number)
{
  unsigned char *chptr;
  chptr = (unsigned char *) &number;
  for(int i = 0; i<sizeof(double); i++){
    *chptr = buf[i] - '0';
    chptr++;
  }
}

void CharToFloat(char* buf, float &number)
{
  unsigned char *chptr;
  chptr = (unsigned char *) &number;
  for(int i = 0; i<sizeof(float); i++){
    *chptr = buf[i] - '0';
    chptr++;
  }
}

bool checkValue(double var, double min, double max)
{
  if(var>=min && var<=max)
    return true;
  return false;
}

int get_target_number(char* buf)
{
  int number = CharToInt(buf[1]);
  return number;
}

int get_command_type(char* buf)
{
  int number = CharToInt(buf[2]);
  return number;
}

bool checksum(char* buf)
{
  int length = strlen(buf+1);
  unsigned char sum = 0;
  for(int i = 1; i < length; i++){
    sum += buf[i];
  }
  if((unsigned char)buf[0] == sum)
    return true;
  else
    return false;
}

void form_checksum(char* buf)
{
  int length = strlen(buf+1);
  unsigned char sum = 0;
  for(int i = 1; i < length; i++){
    sum += buf[i];
  }
  buf[0] = sum;
}

std_msgs::Bool get_takeoff_command(char* buf)
{
  std_msgs::Bool result;
  if(CharToInt(buf[3]) == 0)
    result.data = false ;
  else
    result.data = true;
  return result;
}

int get_waypoint_number(char* buf)
{
  return CharToInt(buf[3]);
}

void get_waypoint_info(char* buf, enif_iuc::WaypointTask &waypoint_list)
{
  double velocity, damping_distance;
  CharToDouble(buf+4, velocity);
  waypoint_list.velocity = velocity;
  CharToDouble(buf+12, damping_distance);
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
      CharToDouble(buf+20+byte_number, latitude);
      waypoint.latitude = latitude;
      byte_number += sizeof(double);
      // Get longitude
      CharToDouble(buf+20+byte_number, longitude);
      waypoint.longitude = longitude;
      byte_number += sizeof(double);
      // Get waypoint height
      CharToDouble(buf+20+byte_number, target_height);
      waypoint.target_height = target_height;
      byte_number += sizeof(double);
      // Get staytime
      waypoint.staytime = CharToInt(buf[20+byte_number]);
      byte_number++;
      waypoint_list.mission_waypoint.push_back(waypoint);
    }
}

void get_box(char* buf, std_msgs::Float64MultiArray &box)
{
  double latitude, longitude, width, height, angle;
  double staytime, wp_height, velocity, wp_radius;
  CharToDouble(buf+3,  longitude);
  CharToDouble(buf+11, latitude);
  CharToDouble(buf+19, width);
  CharToDouble(buf+27, height);
  angle     = CharToInt(buf[28]);
  staytime  = CharToInt(buf[29]);
  wp_height = CharToInt(buf[30]);
  velocity  = CharToInt(buf[31]);
  wp_radius = CharToInt(buf[32]);
  box.data.push_back(longitude);
  box.data.push_back(latitude);
  box.data.push_back(width);
  box.data.push_back(height);
  box.data.push_back(angle);
  box.data.push_back(staytime);
  box.data.push_back(wp_height);
  box.data.push_back(velocity);
  box.data.push_back(wp_radius);
}

bool extract_GPS_from_MPS(mps_driver::MPS mps_read)
{
  if(checkValue(mps_read.GPS_latitude, -180, 180) &&
     checkValue(mps_read.GPS_longitude, -180, 180) &&
     checkValue(mps_read.GPS_altitude, 0, 40)){
    gps.latitude = mps_read.GPS_latitude;
    gps.longitude = mps_read.GPS_longitude;
    height.range = mps_read.GPS_altitude;
    return true;
  }
  return false;
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
  float percentLEL, temperature, pressure, humidity;
  double GPS_latitude, GPS_longitude;
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

void clearmps()
{
  mps.percentLEL = 0;
  mps.temperature = 0;
  mps.pressure = 0;
  mps.humidity = 0;
}
