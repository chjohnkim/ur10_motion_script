# ur10_motion_script

This is a ROS package to simulate UR10 on gazebo and control the simulation via Moveit! with rviz or a python script.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Hardware Prerequisites
Hong the UR10
Computer
-Configure your computer network for a static IP address
```
1. Go to network > Edit connections > Add Ehternet Connection
2. IPv4 Settings: (Method = Manual, Address = 192.168.1.111, Netmask = 24, Gateway = 192.168.1.1
3. Save
```


### Software Prerequisites
- ROS kinetic 
- Universal robot package (communication with UR10 controllers)
- Moveit! 
- Gazebo
- Rviz
- Software tested on Ubuntu 16.04.3 LTS.

### Installing
#### ur_modern_driver setup
1. Open terminal and go to your ~/catkin workspace/src directory
2. git clone modern driver
```
git clone <clone url>
```
3. Build catkin workspace
4. If there is error in building, try replacing <ur_hardware_interface.cpp> file in the src directory of ur_modern_driver package (~/catkin_ws/src/ur_modern_driver/src)

## How to run it

1. Run a gazebo simulation by launching the following commands in three different terminals
```
roscore
```
```
roslaunch ur_gazebo ur10.launch
```
```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
```
2. Rviz can also be executed with the following command on a fourth terminal
```
roslaunch ur10_moveit_config moveit_rviz.launch config:=true
```

3. Run the executable python motion plan to run simulation in gazebo environment:
```
rosrun ur10_motion_script ur10_turnArcFunction.py
```

### Functions

1. TurnArcAboutAxis(axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree, direction, tilt, tilt_axis, tilt_direction)
Purpose: Turns about a reference center point in path mode or tilt mode 
Parameters: 
- axis: ['x'/'y'/'z'] - reference axis to turn the position of the end effector about
- CenterOfCircle_1,2: [float] - coordinate of the reference center of circle to turn the end effector about
- angle_degree: [degrees] - angular distance to turn
- direction: [1/-1] - 1 to turn ccw, -1 to turn cw
- tilt: ['yes'/'no'] - tilt mode yes or no
- tilt_axis: ['x'/'y'/'z'] - tilting axis with respect to the end effector frame
- tilt_direction: [1/-1] - 1 to turn ccw, -1 to turn cw 

2. assign_joint_value(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
Purpose: Manipulate robot arm by specifying joint values
Parameters: 
- joint angles in radians; joint order from base to wrist 

3. assign_pose_target(orient_x, orient_y, orient_z, orient_w, pos_x, pos_y, pos_z)
Purpose: Manipulate robot arm by specifying end-effector frame pose
Parameters: 
- Orientation in quaternions with respect to end-effector frame
- Cartesian coordinates of end-effector with respect to world frame

## Authors

* **John Kim** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Hat tip to anyone who's code was used
* Inspiration
* etc
