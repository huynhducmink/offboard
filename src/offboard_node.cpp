#include <offboard/offboard.h>
#include <offboard/logging.h>

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "offboard");
    ros::NodeHandle nh;

    // subscriber
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber batt_sub = nh.subscribe<sensor_msgs::BatteryState> 
            ("mavros/battery", 10, battery_cb);

    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, localPose_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix> 
            ("mavros/global_position/global", 10, globalPosition_cb);
    ros::Subscriber gps_pos_sub = nh.subscribe<mavros_msgs::GPSRAW> 
            ("mavros/gpsstatus/gps1/raw", 10, gpsPosition_cb);
    ros::Subscriber rel_alt_sub = nh.subscribe<std_msgs::Float64>
            ("mavros/global_position/rel_alt", 10, relativeAlt_cb);
    
    // service
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
			("mavros/set_mode");
    ros::ServiceClient disarming_client = nh.serviceClient<mavros_msgs::CommandBool> 
            ("mavros/cmd/arming");

    // publisher
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
	{
        std::cout << "[ INFO] Waiting for FCU connection...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] FCU connected \n";

    // wait for GPS information
    while (ros::ok() && !global_position_received && !gps_position_received) 
    {
        std::cout << "[ INFO] Waiting for GPS signal...\n";
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] GPS position received \n";
    std::cout << "[ INFO] Waiting for stable initial... \n";
    
    t_check = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - t_check) < ros::Duration(20))
    {
        gps_lat = double(gps_position.lat)/10000000;
        gps_lon = double(gps_position.lon)/10000000;
        gps_alt = double(gps_position.alt)/1000;

        enu_curr = WGS84ToENU(global_position.latitude,
                          global_position.longitude,
                          global_position.altitude,
                          refpoint.latitude, 
                          refpoint.longitude, 
                          refpoint.altitude);
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "[ INFO] Stable initial done \n";
    ros::Duration(1).sleep();
    
    // init reference point
    refpoint.latitude = global_position.latitude;
    refpoint.longitude = global_position.longitude;
    refpoint.altitude = global_position.altitude;

    std::printf("\nCurrent global position: [%f, %f, %.3f]\n", 
                global_position.latitude, 
                global_position.longitude, 
                global_position.altitude);
    std::cout << "Current relative altitude: " << rel_alt.data << " m \n";
        
    std::printf("Reference GPS position: [%f, %f, %.3f]\n", 
                    refpoint.latitude, 
                    refpoint.longitude, 
                    refpoint.altitude);
    enu_curr = WGS84ToENU(global_position.latitude,
                        global_position.longitude,
                        global_position.altitude,
                        refpoint.latitude, 
                        refpoint.longitude, 
                        refpoint.altitude);
    enu_ref = enu_curr;
    std::printf("Current local position: [%.3f, %.3f, %.3f]\n", 
                current_pose.pose.position.x, 
                current_pose.pose.position.y, 
                current_pose.pose.position.z);
    std::printf("Current converted position: [%.3f, %.3f, %.3f]\n", 
                enu_curr.x, 
                enu_curr.y, 
                enu_curr.z);

    for(int i = 100; ros::ok() && i > 0; --i)
    {
        enu_curr = WGS84ToENU(global_position.latitude,
                            global_position.longitude,
                            global_position.altitude,
                            refpoint.latitude, 
                            refpoint.longitude, 
                            refpoint.altitude);
        x_off[i] = current_pose.pose.position.x - enu_curr.x;
        y_off[i] = current_pose.pose.position.y - enu_curr.y;
        z_off[i] = current_pose.pose.position.z - enu_curr.z;

        ros::spinOnce();
        rate.sleep();
    }
    for(int i = 100; i > 0; --i)
    {
        x_offset = x_offset + x_off[i]/100;
        y_offset = y_offset + y_off[i]/100;
        z_offset = z_offset + z_off[i]/100;
    }
    std::printf("\nOffset: [%f, %f, %f]\n", x_offset, y_offset, z_offset);
    
    // set target pose
    input_target();
    if (input_type == true) // local setpoint
    {
        target_pose.pose.position.x = target_pos[0][0];
        target_pose.pose.position.y = target_pos[0][1];
        target_pose.pose.position.z = target_pos[0][2];
    }
    else // global setpoint
    {
        enu_goal = WGS84ToENU(goal_pos[0][0], 
                              goal_pos[0][1], 
                              goal_pos[0][2],
                              refpoint.latitude, 
                              refpoint.longitude, 
                              refpoint.altitude);
        target_pose.pose.position.x = enu_goal.x + x_offset;
        target_pose.pose.position.y = enu_goal.y + y_offset;
        target_pose.pose.position.z = enu_goal.z + z_offset;
    }

    // send a few setpoints before starting
    std::cout << "[ INFO] Setting OFFBOARD stream...\n";
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        target_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Set OFFBOARD stream done \n";    
    std::cout << "[ INFO] Waiting arm and takeoff... \n";
    while (ros::ok() && !current_state.armed)
    {
        ros::spinOnce();
        rate.sleep();
    }

    system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
	ros::param::get("xt", takeoff_pose.pose.position.x);
	ros::param::get("yt", takeoff_pose.pose.position.y);
	ros::param::get("zt", takeoff_pose.pose.position.z);
    ros::param::get("hover_time", t_hover);
    std::cout << "[ INFO] Takeoff... \n";
    bool takeoff_check = false;
    while (ros::ok() && !takeoff_check)
    {
        takeoff_pose.header.stamp = ros::Time::now();   
        local_pos_pub.publish(takeoff_pose);
                    
        takeoff_check = check_position(check_error,
                                    takeoff_pose.pose.position.x,
                                    takeoff_pose.pose.position.y,
                                    takeoff_pose.pose.position.z,
                                    current_pose.pose.position.x,
                                    current_pose.pose.position.y,
                                    current_pose.pose.position.z);
        t_check = ros::Time::now();
        if (takeoff_check)
        {
            while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
            {
                local_pos_pub.publish(takeoff_pose);

                ros::spinOnce();
    		    rate.sleep();
            }
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
        
    }

    int i = 0;
    if (input_type) // local setpoints
    {
        while (ros::ok())
        {
            if (i < (target_num -1))
            {
                final_check = false;
                target_pose.pose.position.x = target_pos[i][0];
                target_pose.pose.position.y = target_pos[i][1];
                target_pose.pose.position.z = target_pos[i][2];
            
                target_pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(target_pose);

        		ros::spinOnce();
                rate.sleep();
            }
            else
            {
                final_check = true;
                target_pose.pose.position.x = target_pos[target_num - 1][0];
                target_pose.pose.position.y = target_pos[target_num - 1][1];
                target_pose.pose.position.z = target_pos[target_num - 1][2];
            
                target_pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(target_pose);

        		ros::spinOnce();
                rate.sleep();
            }
            std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                         current_pose.pose.position.x, 
                         current_pose.pose.position.y, 
                         current_pose.pose.position.z);
            std::printf("Target local position: [%.3f, %.3f, %.3f]\n", 
                                target_pos[i][0], 
                                target_pos[i][1],
                                target_pos[i][2]);
            distance = distanceLocal(current_pose.pose.position.x, 
                                     current_pose.pose.position.y, 
                                     current_pose.pose.position.z,
                                     target_pos[i][0], 
                                     target_pos[i][1], 
                                     target_pos[i][2]);
            std::printf("Distance to target: %.3f m \n", distance);

            bool check = check_position(check_error,
                                        target_pose.pose.position.x,
                                        target_pose.pose.position.y,
                                        target_pose.pose.position.z,
                                        current_pose.pose.position.x,
                                        current_pose.pose.position.y,
                                        current_pose.pose.position.z);
            std::cout << "\n" << check << std::endl;
            if(check && !final_check)
            {
                system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                ros::param::get("hover_time", t_hover);

                std::printf("\n[ INFO] Reached position: [%.3f, %.3f, %.3f]\n", 
                                current_pose.pose.position.x, 
                                current_pose.pose.position.y, 
                                current_pose.pose.position.z);   
                std::printf("[ INFO] Next local position: [%.3f, %.3f, %.3f]\n", 
                                target_pos[i+1][0], 
                                target_pos[i+1][1],
                                target_pos[i+1][2]);
                std::cout << "[ INFO] Ready to land for cargo unpacking \n";

                t_check = ros::Time::now();
                while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
                {
                    local_pos_pub.publish(target_pose);

                    ros::spinOnce();
    		        rate.sleep();
                }
                
                std::cout << "[ INFO] Height is Descending...\n";

                system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                ros::param::get("descend_range", descend_range);
                waypoint_pose.pose.position.x = target_pose.pose.position.x;
                waypoint_pose.pose.position.y = target_pose.pose.position.y;
                waypoint_pose.pose.position.z = target_pose.pose.position.z;
                z_hover = waypoint_pose.pose.position.z;
                num_waypoints = 0;
                if(!in_z_landing.empty())
                {
                    in_z_landing.clear();
                }
                while((z_hover - descend_range) > 0)
                {
                    z_hover = z_hover - descend_range;
                    in_z_landing.push_back(z_hover);
                    num_waypoints++;
                }
                in_z_landing.push_back(0);
                num_waypoints++;
                int j = 0;
                while(j < num_waypoints) //ros::ok() &&
                {
                    target_pose.pose.position.z = in_z_landing[j];
                    target_pose.header.stamp = ros::Time::now();   
                    local_pos_pub.publish(target_pose);

                    system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                    ros::param::get("trigger", trigger);

                    if(trigger)
                    {
                        std::cout << "\nHuman detected\n";
                        j = 0;
                        ros::spinOnce();
                        rate.sleep();
                    }
                    else
                    {
                        bool waypoint_check = check_position(check_error,
                                                    target_pose.pose.position.x,
                                                    target_pose.pose.position.y,
                                                    target_pose.pose.position.z,
                                                    current_pose.pose.position.x,
                                                    current_pose.pose.position.y,
                                                    current_pose.pose.position.z);
                        if(waypoint_check)
                        {            
                            system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                            ros::param::get("waypoint_delay", t_delay);
                            t_check = ros::Time::now();
                            while((ros::Time::now() - t_check) < ros::Duration(t_delay))
                            {
                                local_pos_pub.publish(target_pose);
                                ros::spinOnce();
                                rate.sleep();
                            }
                            std::cout << "Height is Descending...\n";
                            j++;
                        }
                        else
                        {
                            ros::spinOnce();
                            rate.sleep();
                        }
                    }   
                }

                std::cout << "[ INFO] Cargo unpacking\n";
                system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                ros::param::get("cargo_unpack", t_cargo);
                target_pose = current_pose;
                t_check = ros::Time::now();
                while ((ros::Time::now() - t_check) < ros::Duration(t_cargo)) 
                {
                    local_pos_pub.publish(target_pose);
                    ros::spinOnce();
                    rate.sleep();                    
                }

                bool waypoint_check = false;
                std::cout << "[ INFO] Return setpoint\n";
                target_pose = waypoint_pose;
                while (!waypoint_check) 
                {
                    target_pose.header.stamp = ros::Time::now();   
                    local_pos_pub.publish(target_pose);
                    
                    waypoint_check = check_position(check_error,
                                                target_pose.pose.position.x,
                                                target_pose.pose.position.y,
                                                target_pose.pose.position.z,
                                                current_pose.pose.position.x,
                                                current_pose.pose.position.y,
                                                current_pose.pose.position.z);
                    if (waypoint_check)
                    {
                        t_check = ros::Time::now();
                        while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
                        {
                            local_pos_pub.publish(target_pose);

                            ros::spinOnce();
                            rate.sleep();
                        }
                    }
                    ros::spinOnce();
                    rate.sleep();
                }
                
                i = i + 1;
                ros::spinOnce();
    		    rate.sleep();
    		}
            else if (check && final_check)
            {
                system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                ros::param::get("hover_time", t_hover);
                
                std::printf("[ INFO] Reached FINAL position: [%.3f, %.3f, %.3f]\n", 
                                current_pose.pose.position.x, 
                                current_pose.pose.position.y, 
                                current_pose.pose.position.z);
                std::printf("[ INFO] Ready to LANDING \n");

                t_check = ros::Time::now();
                while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
                {
                    local_pos_pub.publish(target_pose);

                    ros::spinOnce();
    		        rate.sleep();
                }

                std::cout << "[ INFO] Height is Descending...\n";
                system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                ros::param::get("descend_range", descend_range);
                waypoint_pose.pose.position.x = target_pose.pose.position.x;
                waypoint_pose.pose.position.y = target_pose.pose.position.y;
                waypoint_pose.pose.position.z = target_pose.pose.position.z;
                z_hover = waypoint_pose.pose.position.z;
                num_waypoints = 0;
                if(!in_z_landing.empty())
                {
                    in_z_landing.clear();
                }
                while((z_hover - descend_range) > 1)
                {
                    z_hover = z_hover - descend_range;
                    in_z_landing.push_back(z_hover);
                    num_waypoints++;
                }
                int j = 0;
                while(j < num_waypoints) 
                {
                    target_pose.pose.position.z = in_z_landing[j];
                    target_pose.header.stamp = ros::Time::now();   
                    local_pos_pub.publish(target_pose);

                    system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                    ros::param::get("trigger", trigger);

                    if(trigger)
                    {
                        std::cout << "\nHuman detected\n";
                        j = 0;
                        ros::spinOnce();
                        rate.sleep();
                    }
                    else
                    {
                        bool waypoint_check = check_position(check_error,
                                                    target_pose.pose.position.x,
                                                    target_pose.pose.position.y,
                                                    target_pose.pose.position.z,
                                                    current_pose.pose.position.x,
                                                    current_pose.pose.position.y,
                                                    current_pose.pose.position.z);
                        if(waypoint_check)
                        {            
                            system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                            ros::param::get("waypoint_delay", t_delay);
                            t_check = ros::Time::now();
                            while((ros::Time::now() - t_check) < ros::Duration(t_delay))
                            {
                                local_pos_pub.publish(target_pose);
                                ros::spinOnce();
                                rate.sleep();
                            }
                            std::cout << "Height is Descending...\n";
                            j++;
                        }
                        else
                        {
                            ros::spinOnce();
                            rate.sleep();
                        }
                    }   
                }

                set_mode.request.custom_mode = "AUTO.LAND";
                if( set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                {
                    std::cout << "[ INFO] AUTO.LAND enabled \n";
                    break;
                }
            }
    		else 
    		{
    			ros::spinOnce();
    		    rate.sleep();
    		} 

            ros::spinOnce();
            rate.sleep();
        }
        
    }
    else // global setpoints
    {
        while (ros::ok())
        {
            if (i < (goal_num - 1))
            {
                final_check = false;
                enu_goal = WGS84ToENU(goal_pos[i][0], 
                                      goal_pos[i][1], 
                                      goal_pos[i][2],
                                      refpoint.latitude, 
                                      refpoint.longitude, 
                                      refpoint.altitude);
                target_pose.pose.position.x = enu_goal.x + x_offset;
                target_pose.pose.position.y = enu_goal.y + y_offset;
                target_pose.pose.position.z = enu_goal.z + z_offset;
                
                target_pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(target_pose);

                ros::spinOnce();
                rate.sleep();
            }
            else
            {
                final_check = true;
                enu_goal = WGS84ToENU(goal_pos[goal_num-1][0], 
                                      goal_pos[goal_num-1][1], 
                                      goal_pos[goal_num-1][2],
                                      refpoint.latitude, 
                                      refpoint.longitude, 
                                      refpoint.altitude);
                target_pose.pose.position.x = enu_goal.x + x_offset;
                target_pose.pose.position.y = enu_goal.y + y_offset;
                target_pose.pose.position.z = enu_goal.z + z_offset;
                
                target_pose.header.stamp = ros::Time::now();
                local_pos_pub.publish(target_pose);
 
                ros::spinOnce();
                rate.sleep();
            }
            std::printf("\nCurrent GPS position: [%.8f, %.8f, %.3f]\n", 
                        global_position.latitude, 
                        global_position.longitude, 
                        global_position.altitude);
            std::printf("Goal GPS position: [%.8f, %.8f, %.3f]\n", 
                                goal_pos[i][0], 
                                goal_pos[i][1],
                                goal_pos[i][2]);
           
            std::printf("\nCurrent local position: [%.3f, %.3f, %.3f]\n", 
                         current_pose.pose.position.x, 
                         current_pose.pose.position.y, 
                         current_pose.pose.position.z);
            std::printf("Target local position: [%.3f, %.3f, %.3f]\n", 
                                target_pose.pose.position.x, 
                                target_pose.pose.position.y,
                                target_pose.pose.position.z);
            distance = distanceLocal(current_pose.pose.position.x, 
                                     current_pose.pose.position.y, 
                                     current_pose.pose.position.z,
                                     target_pose.pose.position.x, 
                                     target_pose.pose.position.y, 
                                     target_pose.pose.position.z);
            std::printf("Distance to target: %.3f m \n", distance);

            bool check = check_position(check_error,
                                        target_pose.pose.position.x,
                                        target_pose.pose.position.y,
                                        target_pose.pose.position.z,
                                        current_pose.pose.position.x,
                                        current_pose.pose.position.y,
                                        current_pose.pose.position.z);

            std::cout << "\n" << check << std::endl;
            if (check && !final_check)
            {
                system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                ros::param::get("hover_time", t_hover);

                std::printf("[ INFO] Reached position: [%.8f, %.8f, %.3f]\n", 
                                global_position.latitude, 
                                global_position.longitude, 
                                global_position.altitude);
                std::printf("[ INFO] Next GPS position: [%.8f, %.8f, %.3f]\n", 
                                goal_pos[i+1][0], 
                                goal_pos[i+1][1],
                                goal_pos[i+1][2]);
                std::printf("[ INFO] Reached position: [%.3f, %.3f, %.3f]\n", 
                                current_pose.pose.position.x, 
                                current_pose.pose.position.y, 
                                current_pose.pose.position.z);   
                std::printf("[ INFO] Target local position: [%.3f, %.3f, %.3f]\n", 
                                target_pose.pose.position.x, 
                                target_pose.pose.position.y,
                                target_pose.pose.position.z);
                std::cout << "[ INFO] Ready to land for cargo unpacking \n";
                
                t_check = ros::Time::now();
                while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
                {
                    local_pos_pub.publish(target_pose);

                    ros::spinOnce();
    		        rate.sleep();
                }
                
                std::cout << "[ INFO] Height is Descending...\n";

                system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                ros::param::get("descend_range", descend_range);
                waypoint_pose.pose.position.x = target_pose.pose.position.x;
                waypoint_pose.pose.position.y = target_pose.pose.position.y;
                waypoint_pose.pose.position.z = target_pose.pose.position.z;
                z_hover = waypoint_pose.pose.position.z;
                num_waypoints = 0;
                if(!in_z_landing.empty())
                {
                    in_z_landing.clear();
                }
                while((z_hover - descend_range) > 0)
                {
                    z_hover = z_hover - descend_range;
                    in_z_landing.push_back(z_hover);
                    num_waypoints++;
                }
                in_z_landing.push_back(0);
                num_waypoints++;
                int j = 0;
                while(j < num_waypoints) //ros::ok() &&
                {
                    target_pose.pose.position.z = in_z_landing[j];
                    target_pose.header.stamp = ros::Time::now();   
                    local_pos_pub.publish(target_pose);

                    system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                    ros::param::get("trigger", trigger);

                    if(trigger)
                    {
                        std::cout << "\nHuman detected\n";
                        j = 0;
                        ros::spinOnce();
                        rate.sleep();
                    }
                    else
                    {
                        bool waypoint_check = check_position(check_error,
                                                    target_pose.pose.position.x,
                                                    target_pose.pose.position.y,
                                                    target_pose.pose.position.z,
                                                    current_pose.pose.position.x,
                                                    current_pose.pose.position.y,
                                                    current_pose.pose.position.z);
                        if(waypoint_check)
                        {            
                            system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                            ros::param::get("waypoint_delay", t_delay);
                            t_check = ros::Time::now();
                            while((ros::Time::now() - t_check) < ros::Duration(t_delay))
                            {
                                local_pos_pub.publish(target_pose);
                                ros::spinOnce();
                                rate.sleep();
                            }
                            std::cout << "Height is Descending...\n";
                            j++;
                        }
                        else
                        {
                            ros::spinOnce();
                            rate.sleep();
                        }
                    }   
                }

                std::cout << "[ INFO] Cargo unpacking\n";
                system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                ros::param::get("cargo_unpack", t_cargo);
                target_pose = current_pose;
                t_check = ros::Time::now();
                while ((ros::Time::now() - t_check) < ros::Duration(t_cargo)) 
                {
                    local_pos_pub.publish(target_pose);
                    ros::spinOnce();
                    rate.sleep();                    
                }

                bool waypoint_check = false;
                std::cout << "[ INFO] Return setpoint\n";
                target_pose = waypoint_pose;
                while (!waypoint_check) 
                {
                    target_pose.header.stamp = ros::Time::now();   
                    local_pos_pub.publish(target_pose);
                    
                    waypoint_check = check_position(check_error,
                                                target_pose.pose.position.x,
                                                target_pose.pose.position.y,
                                                target_pose.pose.position.z,
                                                current_pose.pose.position.x,
                                                current_pose.pose.position.y,
                                                current_pose.pose.position.z);
                    if (waypoint_check)
                    {
                        t_check = ros::Time::now();
                        while ((ros::Time::now() - t_check) < ros::Duration(t_hover))
                        {
                            local_pos_pub.publish(target_pose);

                            ros::spinOnce();
                            rate.sleep();
                        }
                    }
                    ros::spinOnce();
                    rate.sleep();
                }

                i = i + 1;
                ros::spinOnce();
                rate.sleep();
            }
            else if (check && final_check)
            {
                system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                ros::param::get("hover_time", t_hover);
                
                std::printf("[ INFO] Reached FINAL position: [%.8f, %.8f, %.3f]\n", 
                                global_position.latitude, 
                                global_position.longitude, 
                                global_position.altitude);
                std::printf("[ INFO] Reached local position: [%.3f, %.3f, %.3f]\n", 
                                current_pose.pose.position.x, 
                                current_pose.pose.position.y, 
                                current_pose.pose.position.z);
                std::printf("[ INFO] Target local position: [%.3f, %.3f, %.3f]\n", 
                                target_pose.pose.position.x, 
                                target_pose.pose.position.y,
                                target_pose.pose.position.z);
                std::printf("[ INFO] Ready to LANDING \n");
                
                t_check = ros::Time::now();
                while ((ros::Time::now() - t_check) < ros::Duration(10))
                {
                    local_pos_pub.publish(target_pose);

                    ros::spinOnce();
    		        rate.sleep();
                }
                
                std::cout << "[ INFO] Height is Descending...\n";
                system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                ros::param::get("descend_range", descend_range);
                waypoint_pose.pose.position.x = target_pose.pose.position.x;
                waypoint_pose.pose.position.y = target_pose.pose.position.y;
                waypoint_pose.pose.position.z = target_pose.pose.position.z;
                z_hover = waypoint_pose.pose.position.z;
                num_waypoints = 0;
                if(!in_z_landing.empty())
                {
                    in_z_landing.clear();
                }
                while((z_hover - descend_range) > 1)
                {
                    z_hover = z_hover - descend_range;
                    in_z_landing.push_back(z_hover);
                    num_waypoints++;
                }
                int j = 0;
                while(j < num_waypoints)
                {
                    target_pose.pose.position.z = in_z_landing[j];
                    target_pose.header.stamp = ros::Time::now();   
                    local_pos_pub.publish(target_pose);

                    system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                    ros::param::get("trigger", trigger);

                    if(trigger)
                    {
                        std::cout << "\nHuman detected\n";
                        j = 0;
                        ros::spinOnce();
                        rate.sleep();
                    }
                    else
                    {
                        bool waypoint_check = check_position(check_error,
                                                    target_pose.pose.position.x,
                                                    target_pose.pose.position.y,
                                                    target_pose.pose.position.z,
                                                    current_pose.pose.position.x,
                                                    current_pose.pose.position.y,
                                                    current_pose.pose.position.z);
                        if(waypoint_check)
                        {            
                            system("rosparam load $HOME/ros/catkin_ws/src/offboard/config/waypoints.yaml");
                            ros::param::get("waypoint_delay", t_delay);
                            t_check = ros::Time::now();
                            while((ros::Time::now() - t_check) < ros::Duration(t_delay))
                            {
                                local_pos_pub.publish(target_pose);
                                ros::spinOnce();
                                rate.sleep();
                            }
                            std::cout << "Height is Descending...\n";
                            j++;
                        }
                        else
                        {
                            ros::spinOnce();
                            rate.sleep();
                        }
                    }   
                }

                set_mode.request.custom_mode = "AUTO.LAND";
                if( set_mode_client.call(set_mode) && set_mode.response.mode_sent)
                {
                    std::cout << "[ INFO] AUTO.LAND enabled \n";
                    break;
                }
            }
            else 
            {
                ros::spinOnce();
                rate.sleep();
            } 

            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}
