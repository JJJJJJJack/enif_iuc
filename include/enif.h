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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "signal_sub_pub/Signal.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "enif_iuc/Waypoint.h"
#include "enif_iuc/WaypointTask.h"
#include "dji_sdk/dji_drone.h"
#include "mps_driver/MPS.h"

#define COMMAND_TAKEOFF  1
#define COMMAND_WAYPOINT 2
#define COMMAND_GPS      3
#define COMMAND_MPS      4
#define COMMAND_STATE    5

#define GAS_NONE    0
#define GAS_PROPANE 1
#define GAS_METHANE 2

int AGENT_NUMBER = 0;
int GS_ID = 0;
int GAS_ID = 0;

std_msgs::UInt8 state;
dji_sdk::GlobalPosition gps;
mps_driver::MPS mps;

std_msgs::Bool takeoff_command;
enif_iuc::WaypointTask waypoint_list;
