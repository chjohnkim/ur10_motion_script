# ur10_motion_planning

This is a ROS package to simulate UR10 on gazebo and control the simulation via Moveit! with rviz or a python script.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Hardware Prerequisites

ur10

### Software Prerequisites
- ROS kinetic. 
- Universal robot package (communication with UR10 controllers).
- Moveit! 
- Gazebo
- Rviz
- Software tested on Ubuntu 16.04.3 LTS.

### Installing

A step by step series of examples that tell you have to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### Functions

1. TurnArcAboutAxis(axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree, direction, tilt, tilt_axis, tilt_direction)
Purpose: Turns about a reference center point in path mode or tilt mode 
Parameters: 
- axis: ['x'/'y'/'z'] - reference axis to turn the position of the end effector about
- CenterOfCircle_1,2: [int] - coordinate of the reference center of circle to turn the end effector about
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
