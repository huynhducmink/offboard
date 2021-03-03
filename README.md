# ivsr offboard package
#### Local/Global Position control and Human-avoid Landing
## I. Overview
- *include/offboard/offboard.h* : header offboard
- *include/offboard/logging.h*  : header logging

- *src/hover_node.cpp*      : keep drone hovering on input z height
- *src/offboard_node.cpp*   : keep drone flying follow setpoints (local or global), landing at each setpoint to unpack cargo with avoid human
- *src/logging_node.cpp*    : get data and write into "position.csv", "sensor.csv" files that at current working directory
- *src/offboard_lib.cpp*    : library for offboard node
- *src/logging_lib.cpp*     : library for logging node
- *src/setmode_offb.cpp*    : set OFFBOARD mode and ARM vehicle in simulation

- *config/waypoints.yaml*   : prepared waypoints to load into offboard node
- *package.xml*             : ros manifests
- *CMakeLists.txt*          : CMakeLists

## II. Required
- **ROS**             : tested on Melodic (Ubuntu 18.04)
- **PX4**             : tested on v10.0.1 
- **catkin workspace**: `catkin_ws`
- **MAVROS**          : [here](https://dev.px4.io/master/en/ros/mavros_installation.html)

- **git clone `offboard` to `catkin_ws/src/` and build `catkin build`**

## III. Usage

### 1. PRACTICE
#### 1.0. ssh to jetson nano
- `ssh ivsr-nano@192.168.x.x`

- **run in each ground PC's terminal to connect Jetson**
#### 1.1. connect to pixhawk 4 (terminal 1)
- `roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600`
 
- *change landing auto disarm parameter*: `pxh> param set COM_DISARM_LAND -1`

#### 1.3. run code on jetson (terminal 2)
##### **run 1.3.1 or 1.3.2 with each difference purpose**
##### 1.3.1. hovering node
- *run hover_node*                 : `rosrun offboard hover`
- **check current position on screen**

  **input target height for hovering (in meter): z**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

##### 1.3.2. offboard node
- *run offboard_node*                 : `rosrun offboard offboard`
- **manual input or load input from waypoints.yaml config file**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

- simualate detected human by change `trigger` parameter in waypoints.yaml configure file from `false` to `true`

##### **after complete, re-change landing auto disarm parameter**
- `pxh> param set COM_DISARM_LAND 2`

### 2. SIMULATION
#### 2.1. run px4 simulation in gazebo 
- `roslaunch px4 mavros_posix_sitl.launch`
#### 2.2. run code
##### 2.2.1. hovering node
- *run hover_node*                 : `rosrun offboard hover`
- **check current position on screen**

  **input target height for hovering (in meter): z**

- *ARM and switch to OFFBOARD mode*: `rosrun offboard setmode_offb`

##### 2.2.2. offboard node
- *run offboard_node*                 : `rosrun offboard offboard`
- **manual input or load input from waypoints.yaml config file**

- *ARM and switch to OFFBOARD mode*: `rosrun offboard setmode_offb`

- simualate detected human by change `trigger` parameter in waypoints.yaml configure file from `false` to `true`

### 3. RUN LOGGING ALONG
- *run logging_node*                 : `rosrun offboard logging`