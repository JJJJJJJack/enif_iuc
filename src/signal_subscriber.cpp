#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <fstream>
#include <signal_sub_pub/Signal.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

#include <time.h>

using namespace std;

nav_msgs::Odometry roomba_odom, vo_odom;
geometry_msgs::PoseStamped slam_pose;
bool Update_r = false, Update_v = false, Update_s = false;

void roomba_callback(const nav_msgs::Odometry &new_message)
{
		Update_r = true;
    roomba_odom = new_message;
}

void vo_callback(const nav_msgs::Odometry &new_message)
{
    vo_odom = new_message;
    Update_v = true;
}

void slam_callback(const geometry_msgs::PoseStamped &new_message)
{
    slam_pose = new_message;
    Update_s = true;
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "signal_subscriber");
  ros::NodeHandle n;
	
  ros::Rate loop_rate(100);
  
  ros::Subscriber subscribe_roomba_command = n.subscribe("iRobot_0/odom",1,roomba_callback);
  ros::Subscriber subscribe_vo_command = n.subscribe("/odom_mono",1,vo_callback);
  ros::Subscriber subscribe_slam_command = n.subscribe("/slam_out_pose",1,slam_callback);
  

  struct timeval tvstart, tvend;
  gettimeofday(&tvstart,NULL);

  int count = 0;
  srand((int)time(0));
  signal_sub_pub::Signal data;
  bool Start_timer = true;

  //ifstream pose_attitude;
  //pose_attitude.open("/home/jack/Dropbox/PhD at Utah/Class/6225/Assignment/Final Project/Motion Planning Final Project/Final_Script/pose_attitude.txt", ios_base::in);
  //if(!pose_attitude){// if it does not work
  //  cerr << "Can't open Data!\n";
   // return 0;
  //}
  ofstream write_file;
  write_file.open("/home/jack/data.dat");
  write_file<<"Time "<<"Roomba_x "<<"Roomba_y "<<"Roomba_z "<<"VO_x "<<"VO_y "<<"VO_z "<<"SLAM_x "<<"SLAM_y "<<"SLAM_z"<<endl;
  
  while (ros::ok())
  {
  	if(Update_r == true){
		write_file<<roomba_odom.header.stamp<<" "<<roomba_odom.pose.pose.position.x<<" "<<roomba_odom.pose.pose.position.y<<" "<<roomba_odom.pose.pose.position.z<<" "<<vo_odom.pose.pose.position.x<<" "<<vo_odom.pose.pose.position.y<<" "<<vo_odom.pose.pose.position.z<<" "<<slam_pose.pose.position.x<<" "<<slam_pose.pose.position.y<<" "<<slam_pose.pose.position.z<<endl;
		Update_r = false;
		}
		if(Update_v == true){
		write_file<<vo_odom.header.stamp<<" "<<roomba_odom.pose.pose.position.x<<" "<<roomba_odom.pose.pose.position.y<<" "<<roomba_odom.pose.pose.position.z<<" "<<vo_odom.pose.pose.position.x<<" "<<vo_odom.pose.pose.position.y<<" "<<vo_odom.pose.pose.position.z<<" "<<slam_pose.pose.position.x<<" "<<slam_pose.pose.position.y<<" "<<slam_pose.pose.position.z<<endl;
		Update_v = false;
		}
		if(Update_s == true){
		write_file<<slam_pose.header.stamp<<" "<<roomba_odom.pose.pose.position.x<<" "<<roomba_odom.pose.pose.position.y<<" "<<roomba_odom.pose.pose.position.z<<" "<<vo_odom.pose.pose.position.x<<" "<<vo_odom.pose.pose.position.y<<" "<<vo_odom.pose.pose.position.z<<" "<<slam_pose.pose.position.x<<" "<<slam_pose.pose.position.y<<" "<<slam_pose.pose.position.z<<endl;
		Update_s = false;
		}

    gettimeofday(&tvend,NULL);
    double totaltime = tvend.tv_sec - tvstart.tv_sec + 1e-6 * (tvend.tv_usec - tvstart.tv_usec);
    //ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
	write_file.close();

  return 0;
}
