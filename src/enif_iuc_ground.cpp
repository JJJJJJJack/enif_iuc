#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <signal_sub_pub/Signal.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
//FIXME MPS message
// callback functions


#include <time.h>

using namespace std;

void state_callback(const std_msgs::Int8 &new_message)
{

}

void mps_callback(const std_msgs::Int8 &new_message)
{

}

void GPS_callback(const std_msgs::Int8 &new_message)
{

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "signal_publisher");
  ros::NodeHandle n;

  ros::Publisher  signal_pub = n.advertise<signal_sub_pub::Signal>("position_attitude", 1000);
  ros::Publisher  wp_pub     = n.advertise<geometry_msgs::PoseStamped>("/crazyflie/goal", 1000);
  ros::Subscriber sub_state  = n.subscribe("state",1,state_callback);
  ros::Subscriber sub_mps    = n.subscribe("mps_data",1,mps_callback);
  ros::Subscriber sub_GPS    = n.subscribe("GPS",  1,GPS_callback);
  
  ros::Rate loop_rate(100);

  struct timeval tvstart, tvend;
  gettimeofday(&tvstart,NULL);

  int count = 0;
  srand((int)time(0));
  enif::Signal data;
  geometry_msgs::PoseStamped goal;

  while (ros::ok())
  {
    signal_pub.publish(data);
    goal_pub.publish(goal);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
