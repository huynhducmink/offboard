#include "offboard/offboard.h"

int main(int argc, char **argv) 
{

    // initialize ros node
    ros::init(argc, argv, "gps");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State> 
            ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 10, globalPosition_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    // service
    ros::ServiceClient set_mav_frame_client = nh.serviceClient<mavros_msgs::SetMavFrame>
			("mavros/setpoint_position/mav_frame");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");

    // publisher
    ros::Publisher goal_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped> 
            ("mavros/setpoint_position/global", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for fcu connection
    while (ros::ok() && !current_state.connected) 
    {
        ROS_INFO_ONCE("Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] FCU connected \n";

    // wait for position information
    while (ros::ok() && !global_position_received) 
    {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] GPS position received \n";
    ros::Duration(1).sleep();

    std::cout << "[ INFO] Checking status... \n";
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_ONCE("\n Current GPS position: [%f, %f, %.3f]", 
                        global_position.latitude, 
                        global_position.longitude, 
                        global_position.altitude);
    std::cout << "[ INFO] Status checked \n";

    // set target position
    input_global_target();
    goal_position.pose.position.latitude = t_latitude;
    goal_position.pose.position.longitude = t_longitude;
    goal_position.pose.position.altitude = t_altitude;
    t_altitude = t_altitude + global_position.altitude;

    geometry_msgs::PoseStamped takeoff_pose;
    takeoff_pose.pose.position.x = 0;
    takeoff_pose.pose.position.y = 0;
    takeoff_pose.pose.position.z = 2.5;
    
    // send a few setpoints before starting
    std::cout << "[ INFO] Setting OFFBOARD stream... \n";
    for (int i=100; ros::ok() && i>0; --i) 
    {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        takeoff_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(takeoff_pose);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] OFFBOARD stream set \n";
    ros::Duration(1).sleep();
        
    while (ros::ok() && !current_state.armed)
    {
        // std::cout << "[ INFO] Waiting arm and takeoff... \n";
        ROS_INFO_ONCE("Waiting arm and takeoff...");
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode set_mode;
    set_mode.request.base_mode = 0;
    // set_mode.request.custom_mode = "AUTO.TAKEOFF";
    // set_mode.request.custom_mode = "AUTO.LAND";
    
    mavros_msgs::SetMavFrame set_mav_frame;
    set_mav_frame.request.mav_frame = 6;
    // set_mav_frame_client.call(set_mav_frame);

    bool reached = false;

    // set_mode.request.custom_mode = "OFFBOARD";
    // set_mode_client.call(set_mode);
    // if (set_mode.response.mode_sent)
    // {
    //     ROS_INFO_ONCE("OFFBOARD enabled \n");
    // }

    while (ros::ok() && !reached)
    {
        takeoff_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(takeoff_pose);

        reached = check_position(takeoff_pose.pose.position.x,
                                 takeoff_pose.pose.position.y,
                                 takeoff_pose.pose.position.z,
                                 current_pose.pose.position.x,
                                 current_pose.pose.position.y,
                                 current_pose.pose.position.z);
        ROS_INFO_ONCE("Takeoff enabled \n");
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Taked-off \n";
    set_mav_frame_client.call(set_mav_frame);
    if (set_mav_frame.response.success)
    {
        ROS_INFO_ONCE("MAV_FRAME set \n");
    }

    reached = false;
    while (ros::ok() && !reached) 
    {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
                                
        ROS_INFO_ONCE("Going to setpoint \n");

        reached = check_global(t_latitude, t_longitude, t_altitude,
                                  global_position.latitude, 
                                  global_position.longitude, 
                                  global_position.altitude);
        ros::spinOnce();
    	rate.sleep();
    }

    ros::Time t_check = ros::Time::now();
    geometry_msgs::PoseStamped hover_pose = current_pose;
    while ((ros::Time::now() - t_check) < ros::Duration(10))
    {
        ROS_INFO_ONCE("Hovering in 10 sec \n");
        local_pos_pub.publish(hover_pose);

        ros::spinOnce();
    	rate.sleep();
    }               

    set_mode.request.custom_mode = "AUTO.LAND";
    set_mode_client.call(set_mode);
    if (set_mode.response.mode_sent)
    {
        ROS_INFO_ONCE("AUTO.LAND enabled \n");
    }
    
    return 0;
}
