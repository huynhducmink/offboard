#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <mavros_msgs/GPSRAW.h>

#include <sensor_msgs/BatteryState.h>

#include <iostream>
#include <cmath>
#include <cstdio>

/****** DEFINE CONSTANTS ******/
const double PI  =3.141592653589793238463;
const double eR  =6378.137; //km

/****** DEFINE FUNCTIONS ******/
bool check_position(void);
bool check_orientation(void);

void input_local_target(void);
void input_global_target(void);

double degree(double);
double radian(double);

double measureGPS(double, double, double, double);

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

int target_num, goal_num;
float target_pos[10][7];
double goal_pos[10][3];

double roll, pitch, yaw;
double r, p, y;

double distance;
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

/****************************************************************************************************/
/***** check_position: check when drone reached the target positions. return the true or false ******/
/****************************************************************************************************/
bool check_position()
{
	bool reached;
	if(((target_pose.pose.position.x - 0.1) < current_pose.pose.position.x)
	&& (current_pose.pose.position.x < (target_pose.pose.position.x + 0.1)) 
	&& ((target_pose.pose.position.y - 0.1) < current_pose.pose.position.y)
	&& (current_pose.pose.position.y < (target_pose.pose.position.y + 0.1))
	&& ((target_pose.pose.position.z - 0.1) < current_pose.pose.position.z)
	&& (current_pose.pose.position.z < (target_pose.pose.position.z + 0.1)))
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

/**********************************************************************************************************/
/***** check_orientation: check when drone reached the target orientations. return the true or false ******/
/**********************************************************************************************************/
bool check_orientation()
{
	bool reached;
	// tf Quaternion to RPY
	tf::Quaternion qc(
		current_pose.pose.orientation.x,
		current_pose.pose.orientation.y,
		current_pose.pose.orientation.z,
		current_pose.pose.orientation.w);
	tf::Matrix3x3 mc(qc);
	double rc, pc, yc;
	mc.getRPY(rc, pc, yc);

	tf::Quaternion qt(
		current_pose.pose.orientation.x,
		current_pose.pose.orientation.y,
		current_pose.pose.orientation.z,
		current_pose.pose.orientation.w);
	tf::Matrix3x3 mt(qt);
	double rt, pt, yt;
	mt.getRPY(rt, pt, yt);
	// check
	if((((degree(rt)-1)<(degree(rc)))&&(degree(rc)<(degree(rt)+1)))
	 &&(((degree(pt)-1)<(degree(pc)))&&(degree(pc)<(degree(pt)+1)))
	 &&(((degree(yt)-1)<(degree(yc)))&&(degree(yc)<(degree(yt)+1)))) 
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

/****************************************************************************************/
/***** check_goal: check when drone reached the GPS goal. return the true or false ******/
/****************************************************************************************/
bool check_goal()
{
	bool reached;
	if((global_position.latitude == goal_position.pose.position.latitude)
	 &&(global_position.longitude == goal_position.pose.position.longitude)
	 &&(global_position.altitude == goal_position.pose.position.altitude))
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

/**************************************************************************/
/***** input_local_target: input the number of waypoints and each point   *
 * coodinates (x, y, z). and input the yaw rotation at each waypoint ******/
/**************************************************************************/
void input_local_target()
{
	std::cout << "Input target(s) position:" << std::endl;
	std::cout << "Number of target(s): "; std::cin >> target_num;
	for (int i = 0; i < target_num; i++)
	{
		std::cout << "Target (" << i+1 << ") position (in meter):" <<std::endl; 
		std::cout << "pos_x_" << i+1 << ":"; std::cin >> target_pos[i][0];
		std::cout << "pos_y_" << i+1 << ":"; std::cin >> target_pos[i][1];
		std::cout << "pos_z_" << i+1 << ":"; std::cin >> target_pos[i][2];

		std::cout << "Target (" << i+1 << ") orientation (in degree):" <<std::endl; 
		target_pos[i][3] = 0;
		target_pos[i][4] = 0;
		std::cout << "yaw_" << i+1 << ":"; std::cin >> target_pos[i][5];
	}
}

/*************************************************************************/
/***** input_global_target: input the number of waypoints and each point *
 * GPS [latitude, longitude, altitude]                              ******/
/*************************************************************************/
void input_global_target()
{
	std::cout << "Input goal position:" << std::endl;
	std::cout << "Number of goal: "; std::cin >> goal_num;
	for (int i = 0; i < goal_num; i++)
	{
		std::cout << "Goal (" << i+1 << ") position:" <<std::endl; 
		std::cout << "Latitude__" << i+1 << " (degree):"; std::cin >> goal_pos[i][0];
		std::cout << "Longitude_" << i+1 << " (degree):"; std::cin >> goal_pos[i][1];
		std::cout << "Altitude__" << i+1 << " (meter.ASL):"; std::cin >> goal_pos[i][2];
	}
}

/**********************************************************/
/***** degree: convert angle from radian to degree ******/
/**********************************************************/
double degree(double rad)
{
	double radian_to_degree = (rad*180)/PI;
	return radian_to_degree;
}

/**********************************************************/
/***** radian: convert angle from degree to radian ******/
/**********************************************************/
double radian(double deg)
{
	double degree_to_radian = (deg*PI)/180;
	return degree_to_radian;
}

/*********************************************************************************************/
/***** measureGPS: measure the distance between 2 GPS points that use haversine formula ******/
/*********************************************************************************************/
double measureGPS(double lat1, double lon1, double alt1, double lat2, double lon2, double alt2)
{
	double flat, plus, distance;
	lat1 = radian(lat1); lon1 = radian(lon1);
	lat2 = radian(lat2); lon2 = radian(lon2);
	flat = 2*eR*asin(sqrt(sin((lat2-lat1)/2)*sin((lat2-lat1)/2)+cos(lat1)*cos(lat2)*sin((lon2-lon1)/2)*sin((lon2-lon1)/2)))*1000; //m
	if (alt1 == alt2)
	{
		distance = flat;
	}
	else
	{
		if 	(alt1 > alt2)
		{
			plus = flat/((alt1/alt2)-1);
			distance = sqrt((flat+plus)*(flat+plus) + alt1*alt1) - sqrt(plus*plus + alt2*alt2);
		}
		else
		{
			plus = flat/((alt1/alt2)-1);
			distance = sqrt((flat+plus)*(flat+plus) + alt2*alt2) - sqrt(plus*plus + alt1*alt1);
		}
	}
	return distance;
}
