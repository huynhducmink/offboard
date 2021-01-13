#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/SetMavFrame.h>
#include <sensor_msgs/BatteryState.h>

#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GPSRAW.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <cmath>
#include <cstdio>

/****** DEFINE CONSTANTS ******/
const double PI  =3.141592653589793238463;
const double eR  =6378.137; //km

/****** DEFINE FUNCTIONS ******/
bool check_position(double, double, double,
				    double, double, double);
bool check_orientation(double, double, double, double,
				       double, double, double, double);
bool check_global(double, double, double,
				  double, double, double);

void input_local_target(void);
void input_global_target(void);

double degree(double);
double radian(double);

double measureGPS(double, double, double, double, double, double);

/****** DECLARE VARIANTS ******/
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped target_pose;

bool global_position_received = false;
bool gps_position_received = false;

sensor_msgs::NavSatFix global_position;
geographic_msgs::GeoPoseStamped goal_position;
mavros_msgs::GPSRAW gps_position;

sensor_msgs::BatteryState current_batt;

tf::Quaternion q;

int target_num;
float target_pos[10][7];
double roll, pitch, yaw;
double r, p, y;

double t_latitude, t_longitude, t_altitude, distance;
float batt_percent;

/****** FUNCTIONS ******/

/***** callback functions *****/
// state callback 
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// local pose callback
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
}

// global position callback
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) 
{
    global_position = *msg;
    global_position_received = true;
}

// gps position callback
void gpsPosition_cb(const mavros_msgs::GPSRAW::ConstPtr& msg) 
{
    gps_position = *msg;
	gps_position_received = true;
}

// battery status callback
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg) 
{
    current_batt = *msg;
}