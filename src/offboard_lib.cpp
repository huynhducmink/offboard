#include <offboard/offboard.h>

OffboardControl::OffboardControl()
{
	// constructor
}

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}
void localPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose_ = *msg;
}
void target_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    target_pub_pose = *msg;
	target_pub_check_ = true;
}
void error_cb(const std_msgs::Float64::ConstPtr& msg)
{
    check_error_ = *msg;
}


void OffboardControl::takeOff(ros::Rate rate)
{
    geometry_msgs::PoseStamped take_off_;
    take_off_.pose.position.x = current_pose_.pose.position.x;
    take_off_.pose.position.y = current_pose_.pose.position.y;
    take_off_.pose.position.z = z_target;
    std::cout << "[ INFO] ----- Takeoff \n";
    
    bool check_takeoff = false;
    while (ros::ok())
    {
        take_off_.header.stamp = ros::Time::now();
        local_pos_pub_.publish(take_off_);
        
        check_takeoff = check_position(check, current_pose_, take_off_);
        if (check_takeoff)
        {
            std::cout << "[ INFO] ----- Hovering \n";
			while (!target_pub_check_)
			{
				hover(t_hover_, take_off_, rate);
				std::cout << "no target detected\n";
			}
			std::cout << "[ INFO] --------------- FLY --------------- \n";
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void OffboardControl::hover(double time, geometry_msgs::PoseStamped target, ros::Rate rate)
{
    ros::Time t_check_;
    t_check_ = ros::Time::now();

    while ((ros::Time::now() - t_check_) < ros::Duration(time))
    {
        local_pos_pub_.publish(target);

        ros::spinOnce();
    	rate.sleep();
    }
}

void OffboardControl::landing(geometry_msgs::PoseStamped setpoint, ros::Rate rate)
{
    bool check_land = false;
    std::cout << "\n[ INFO] ----- Descending \n";
    while (ros::ok() && !check_land)
    {
        std::vector<double> v_land_ = vel_limit(current_pose_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0));

        target_pose_.pose.position.x = current_pose_.pose.position.x + v_land_[0];
        target_pose_.pose.position.y = current_pose_.pose.position.y + v_land_[1];
        target_pose_.pose.position.z = current_pose_.pose.position.z + v_land_[2];

        target_pose_.header.stamp = ros::Time::now();
        local_pos_pub_.publish(target_pose_);
            
        std::printf("----- Descending - %.3f (m)\n", current_pose_.pose.position.z);
        check_land = check_position(check, current_pose_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0));
        if (check_land && !final_check_)
        {
            std::cout << "[ INFO] ----- Unpacking \n";
            hover(t_hover_, targetTransfer(setpoint.pose.position.x, setpoint.pose.position.y, 0), rate);
            std::cout << "[ INFO] Unpacked\n";

            std::cout << "[ INFO] ----- Return setpoint\n";
            bool check_return = false;
            while (ros::ok() && !check_return)
            {
                std::vector<double> v_return_ = vel_limit(current_pose_, setpoint); 
                target_pose_.pose.position.x = current_pose_.pose.position.x + v_return_[0];
                target_pose_.pose.position.y = current_pose_.pose.position.y + v_return_[1];
                target_pose_.pose.position.z = current_pose_.pose.position.z + v_return_[2];

                target_pose_.header.stamp = ros::Time::now();
                local_pos_pub_.publish(target_pose_);

                std::printf("----- Ascending - %.3f (m)/ %.3f\n", current_pose_.pose.position.z, setpoint.pose.position.z);
                check_return = check_position(check, current_pose_, setpoint);
                if (check_return)
                {
                    std::cout << "[ INFO] ----- Hovering \n";
                    hover(t_hover_, setpoint, rate);
                }
                ros::spinOnce();
                rate.sleep();
            }
        }
        else if (check_land && final_check_)
        {
            set_mode_.request.custom_mode = "AUTO.LAND";
            if( set_mode_client_.call(set_mode_) && set_mode_.response.mode_sent)
            {
                std::printf("[ INFO] --------------- LAND ---------------\n");
            }
        }
        else
        {
            ros::spinOnce();
            rate.sleep();
        }
    }
}

void OffboardControl::position_control(ros::NodeHandle nh, ros::Rate rate)
{
    nh_ = nh;
	state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, localPose_cb);
    target_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/target_position", 10, target_cb);
    error_sub_ = nh_.subscribe<std_msgs::Float64>("/check_error_pos", 10, error_cb);
	
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    
    // wait for FCU connection
    std::cout << "[ INFO] ----- Waiting for FCU connection \n";
    while(ros::ok() && !current_state_.connected)
	{
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] FCU connected \n";

    target_pose_.pose.position.x = target_pub_pose.pose.position.x;
    target_pose_.pose.position.y = target_pub_pose.pose.position.y;
    target_pose_.pose.position.z = z_target;

    // send a few setpoints before starting
    std::cout << "[ INFO] ----- Setting OFFBOARD stream \n";
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        target_pose_.header.stamp = ros::Time::now();
        local_pos_pub_.publish(target_pose_);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] Set OFFBOARD stream done \n";

    std::cout << "[ INFO] ----- Waiting OFFBOARD switch \n";
    while (ros::ok() && !current_state_.armed && (current_state_.mode != "OFFBOARD"))
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "[ INFO] --------------- READY --------------- \n";
    
    takeOff(rate);

		while (ros::ok())
        {
                vel_ = vel_limit(current_pose_, targetTransfer(target_pub_pose.pose.position.x, target_pub_pose.pose.position.x, z_target));
                target_pose_.pose.position.x = current_pose_.pose.position.x + vel_[0];
                target_pose_.pose.position.y = current_pose_.pose.position.y + vel_[1];
                target_pose_.pose.position.z = current_pose_.pose.position.z + vel_[2];

                target_pose_.header.stamp = ros::Time::now();
                local_pos_pub_.publish(target_pose_);

            // bool check = check_position(check_error_, current_pose_, targetTransfer(target_pub_pose.pose.position.x, target_pub_pose.pose.position.x, z_target));
            bool check = (check_error_.data < 0.15) ? true:false;
            std::cout << "\n" << check << std::endl;
            if(check)
            {
				geometry_msgs::PoseStamped setpoint_land;
				setpoint_land = target_pub_pose;
                std::printf("[ INFO] Reached FINAL position: [%.3f, %.3f, %.3f]\n", 
                                current_pose_.pose.position.x, 
                                current_pose_.pose.position.y, 
                                current_pose_.pose.position.z);

                std::cout << "\n[ INFO] ----- Hovering - Ready to LAND\n";
                hover(t_land_, targetTransfer(setpoint_land.pose.position.x, setpoint_land.pose.position.x, z_target), rate);

                landing(targetTransfer(setpoint_land.pose.position.x, setpoint_land.pose.position.x, z_target), rate);
                break;
                
                ros::spinOnce();
    		    rate.sleep();
            }
    		else 
    		{
    			ros::spinOnce();
    		    rate.sleep();
    		} 
        }
}

std::vector<double> OffboardControl::vel_limit(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
    double xc = current.pose.position.x;
    double yc = current.pose.position.y;
    double zc = current.pose.position.z;

    double xt = target.pose.position.x;
    double yt = target.pose.position.y;
    double zt = target.pose.position.z;

    double dx = xt - xc;
    double dy = yt - yc;
    double dz = zt - zc;

    double d = sqrt(dx*dx + dy*dy + dz*dz);

    std::vector<double> vel;
    if (!vel.empty())
    {
        vel.clear();
    }
    vel_desired_ = 0.2;   
    
    vel.push_back((dx/d) * vel_desired_);
    vel.push_back((dy/d) * vel_desired_);
    vel.push_back((dz/d) * vel_desired_);

    return vel;
}

bool OffboardControl::check_position(float error, geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped target)
{
	double xt = target.pose.position.x;
	double yt = target.pose.position.y;
	double zt = target.pose.position.z;
	double xc = current.pose.position.x;
	double yc = current.pose.position.y;
	double zc = current.pose.position.z;

	if(((xt - error) < xc) && (xc < (xt + error)) 
	&& ((yt - error) < yc) && (yc < (yt + error))
	&& ((zt - error) < zc) && (zc < (zt + error)))
	{
		return true;
	}
	else
	{
		return false;
	}
}

geometry_msgs::PoseStamped targetTransfer(double x, double y, double z)
{
    geometry_msgs::PoseStamped target;
    target.pose.position.x = x;
    target.pose.position.y = y;
    target.pose.position.z = z;
    return target;
}

OffboardControl::~OffboardControl()
{
	// destructor
}
