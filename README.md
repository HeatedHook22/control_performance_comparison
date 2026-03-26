# ROS2_offboardctrl

A ROS 2 offboard control example for PX4 SITL.  
This project is adapted from the PX4 offboard control example in [`px4_ros_com`](https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp).

## Install / Getting Started

### 1. Prerequisites

Make sure the following are already installed:

- WSL
- PX4 Autopilot v1.15.4
- Gazebo Classic
- ROS 2
- Micro XRCE-DDS Agent (Micro-DDS Agent)
- QGroundControl (QGC)


### 2. Install the project
Assume you don't have a workspace:
```
mkdir -p ros2_ws/src
cd ros2_ws/src  
git clone https://github.com/yliu213/ROS2_offboardctrl.git
git clone -b release/1.15 https://github.com/PX4/px4_msgs.git
cd ..
colcon build
source install/setup.bash  # source the workspace
```
Note: The version of px4_msgs package MUST match the version of px4 version. Otherwise the program will experience some unexpected behavior.

### 3. Modify /.bashrc
```
cd
code ~/.bashrc
```  
add following contents in the end, replace <user> by your user name:
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/heated_2204/nonlin_ctrl_lab/control_performance_comparison/models
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/heated_2204/nonlin_ctrl_lab/control_performance_comparison/plugins
```

### 4. Modify empty.world
```
code ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world
```
Add the following inside <world> block:
```
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>250</update_rate>
</plugin>
```

## Run SITL
### 1. Launch SITL environment
In a new terminal:
```
ros2 launch bare_quad_sim/launch/init.launch.py
ros2 launch bare_quad_sim/launch/init.launch.py world:=dual_iris_sim.world
ros2 launch bare_quad_sim/launch/init.launch.py world:=windy.world
```  
And click the start icon in gazebo

### 2. Open QGroundControl and wait for connection

### 3. Launch offboard control launch script
In a new terminal:
```
ros2 launch bare_quad_sim ctrnode.launch.py
```
