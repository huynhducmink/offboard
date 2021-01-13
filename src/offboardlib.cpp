#include "offboard/offboard.h"


/****************************************************************************************************/
/***** check_position: check when drone reached the target positions. return the true or false ******/
/****************************************************************************************************/
bool check_position(double target_x, double target_y, double target_z,
					double currentx, double currenty, double currentz)
{
	bool reached;
	if(((target_x - 0.1) < currentx)
	&& (currentx < (target_x + 0.1)) 
	&& ((target_y - 0.1) < currenty)
	&& (currenty < (target_y + 0.1))
	&& ((target_z - 0.1) < currentz)
	&& (currentz < (target_z + 0.1)))
	{
		reached = 1;
	}
	else
	{
		reached = 0;
	}
	return reached;
}

bool check_global(double lat, double lon, double alt,
				double lat0, double lon0, double alt0)
{
	bool reached;
	if(
		((lat - 0.000001) < lat0)
	 && (lat0 < (lat + 0.000001)) 
	 && ((lon - 0.000001) < lon0)
	 && (lon0 < (lon + 0.000001))
	 && ((alt - 0.1) < alt0)
	 && (alt0 < (alt + 0.1))
	)
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
bool check_orientation(double target_x, double target_y, double target_z, double target_w,
					   double currentx, double currenty, double currentz, double currentw)
{
	bool reached;
	// tf Quaternion to RPY
	tf::Quaternion qc(currentx,
					  currenty,
					  currentz,
					  currentw);
	tf::Matrix3x3 mc(qc);
	double rc, pc, yc;
	mc.getRPY(rc, pc, yc);

	tf::Quaternion qt(target_x,
					  target_y,
					  target_z,
					  target_w);
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

/*************************************************************************************/
/***** input_global_target: input the GPS [latitude, longitude, altitude] point ******/
/*************************************************************************************/
void input_global_target()
{
	std::cout << "Input GPS position" << std::endl;
	std::cout << "Latitude  (degree):"; std::cin >> t_latitude;
	std::cout << "Longitude (degree):"; std::cin >> t_longitude;
	std::cout << "Altitude  (meter) :"; std::cin >> t_altitude;
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
double measureGPS(double lat1, double lon1, double alt1, 
				  double lat2, double lon2, double alt2)
{
	double flat, plus, Distance;
	lat1 = radian(lat1); lon1 = radian(lon1);
	lat2 = radian(lat2); lon2 = radian(lon2);
	flat = 2*eR*asin(sqrt(sin((lat2-lat1)/2)*sin((lat2-lat1)/2)
	       +cos(lat1)*cos(lat2)*sin((lon2-lon1)/2)*sin((lon2-lon1)/2)))*1000; //m
	alt1 = abs(alt1);
	alt2 = abs(alt2);
	if (alt1 == alt2)
	{
		Distance = flat;
	}
	else
	{
		if 	(alt1 > alt2)
		{
			plus = flat/((alt1/alt2)-1);
			Distance = sqrt((flat+plus)*(flat+plus) + alt1*alt1) 
					   - sqrt(plus*plus + alt2*alt2);
		}
		else
		{
			plus = flat/((alt2/alt1)-1);
			Distance = sqrt((flat+plus)*(flat+plus) + alt2*alt2) 
					   - sqrt(plus*plus + alt1*alt1);
		}
	}
	return Distance;
}