#ifndef OFFBOARD_H
#define OFFBOARD_H

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <std_msgs/Float64.h>
/******* local position *******/
#include <geometry_msgs/PoseStamped.h>

/******* C++ ******/
#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>

bool local_input_ = true; // true == input local || false == input global setpoints
bool final_check_ = true;
bool target_pub_check_ = false;
mavros_msgs::State current_state_; // check connection to pixhawk
geometry_msgs::PoseStamped current_pose_; // current local position 
geometry_msgs::PoseStamped target_pub_pose; 
std_msgs::Float64 check_error_;
geometry_msgs::PoseStamped targetTransfer(double x, double y, double z);

inline void state_cb(const mavros_msgs::State::ConstPtr& msg);
inline void localPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
inline void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
inline void error_cb(const std_msgs::Float64::ConstPtr& msg);

class OffboardControl
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber state_sub_;
    ros::Subscriber local_pose_sub_;
    ros::Subscriber target_pose_sub_;
    ros::Subscriber error_sub_;

    ros::Publisher local_pos_pub_;

    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arming_client_;

    double vel_desired_; // desired velocity
    std::vector<double> vel_;

    mavros_msgs::SetMode set_mode_; // set OFFBOARD mode in simulation
    geometry_msgs::PoseStamped target_pose_; // target local setpoint
    geometry_msgs::PoseStamped setpoint_pose_;
    double z_target = 5;
    double t_hover_ = 3;
	double t_land_ = 3;
    double t_stable_ = 3;
    float check = 0.1;
    bool check_position(float error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);

public:
    OffboardControl();
  
    // void initial_state(ros::NodeHandle nh, ros::Rate rate);
    void takeOff(ros::Rate rate);
    void hover(double time, geometry_msgs::PoseStamped target, ros::Rate rate);
    void landing(geometry_msgs::PoseStamped setpoint, ros::Rate rate);
    void position_control(ros::NodeHandle nh, ros::Rate rate);
    std::vector<double> vel_limit(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target);

    ~OffboardControl();  
};

#endif
