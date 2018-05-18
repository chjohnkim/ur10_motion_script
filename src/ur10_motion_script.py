#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander #important
import moveit_msgs.msg #important
import geometry_msgs.msg #important


moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
rospy.init_node('move_group_python_interface_tutorial', anonymous=True) #initialize the node 

robot = moveit_commander.RobotCommander() #define the robot
scene = moveit_commander.PlanningSceneInterface() #define the scene
group = moveit_commander.MoveGroupCommander("manipulator") #define the planning group (from the moveit packet 'manipulator' planning group)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory) #create a publisher that publishes a plan to the topic: '/move_group/display_planned_path'



group_variable_values = group.get_current_joint_values() #create variable that stores joint values

#Assign values to joints
group_variable_values[0] = 0
group_variable_values[1] = 0
group_variable_values[2] = 0
group_variable_values[3] = 0
group_variable_values[4] = 0
group_variable_values[5] = 0

group.set_joint_value_target(group_variable_values) #set target joint values for 'manipulator' group
 
plan1 = group.plan() #call plan function to plan the path
group.go(wait=True) #execute plan on real/simulation robot
rospy.sleep(2) #sleep 2 seconds

#Assign values to joints
group_variable_values[0] = 1.5
group_variable_values[1] = -1.5
group_variable_values[2] = 1.5
group_variable_values[3] = 0
group_variable_values[4] = 0
group_variable_values[5] = 0

group.set_joint_value_target(group_variable_values) #set target joint values for 'manipulator' group
 
plan1 = group.plan() #call plan function to plan the path
group.go(wait=True) #execute plan on real/simulation robot
rospy.sleep(2) #sleep 2 seconds


pose_target = group.get_current_pose() #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"

#assign values
pose_target.pose.orientation.w = 1.0 
pose_target.pose.orientation.x = 0
pose_target.pose.orientation.y = 0 
pose_target.pose.orientation.z = 0
pose_target.pose.position.x = 0.8 
pose_target.pose.position.y = 0 
pose_target.pose.position.z = 1.1

group.set_pose_target(pose_target) #set pose_target as the goal pose of 'manipulator' group 

plan2 = group.plan() #call plan function to plan the path
group.go(wait=True) #execute plan on real/simulation robot
rospy.sleep(5) #sleep 5 seconds

moveit_commander.roscpp_shutdown() #shut down the moveit_commander


###To retrieve data/status of robot###
#You can get a list with all the groups of the robot like this:
print "Robot Groups:"
print robot.get_group_names()

#You can get the current values of the joints like this:
print "Current Joint Values:"
print group.get_current_joint_values()


#You can also get the current Pose of the end effector of the robot like this:
print "Current Pose:"
print group.get_current_pose()

#Finally you can check the general status of the robot like this:
print "Robot State:"
print robot.get_current_state()

























