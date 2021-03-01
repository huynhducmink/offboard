# ivsr offboard package
#### Local/Global Position control and Human-avoid Landing
## contain
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

## required
- **ros**             : tested on Melodic (Ubuntu 18.04)
- **PX4**             : tested on v10.0.1 
- **catkin workspace**: `catkin_ws`
- **mavros**          : [here](https://dev.px4.io/master/en/ros/mavros_installation.html)

- **git clone `offboard` to `catkin_ws/src/` and build `catkin build`**

## usage

### CONNECT TO PIXHAWK or RUN SIMUALTION
#### on jetson (via ssh)
- *connect jetson to pixhawk* 

- `roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600`
#### run simulation
- `roslaunch px4 mavros_posix_sitl.launch`

#### change landing auto disarm parameter
- `pxh> param set COM_DISARM_LAND -1`

### AFTER CONNECTED TO PIXHAWK or RUN SIMULATION, RUN:
#### hovering node
- *run hover_node*                 : `rosrun offboard hover`
- **check current position on screen**

  **input target height for hovering (in meter): z**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

  on simualation: `rosrun offboard setmode_offb`
### OR:
#### offboard node
- *run offboard_node*                 : `rosrun offboard offboard`
- **manual input or load input from waypoints.yaml config file**
  
- **on remote controller** switch to ARM, then switch flight mode to OFFBOARD

  on simualation: `rosrun offboard setmode_offb`
- simualate detected human by change `trigger` parameter in waypoints.yaml configure file from `false` to `true`

### RUN LOGGING NODE ALONG
#### logging node
- *run logging_node*                 : `rosrun offboard logging`