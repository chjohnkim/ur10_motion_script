#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander #important
import moveit_msgs.msg #important
import geometry_msgs.msg #important

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


#############FUNCTION TO SCALE TRAJECTORY SPEED####################################################
def scale_trajectory_speed(traj, scale):
       # Create a new trajectory object
       new_traj = RobotTrajectory()
      
       # Initialize the new trajectory to be the same as the planned trajectory
       new_traj.joint_trajectory = traj.joint_trajectory
      
       # Get the number of joints involved
       n_joints = len(traj.joint_trajectory.joint_names)
      
       # Get the number of points on the trajectory
       n_points = len(traj.joint_trajectory.points)
       
       # Store the trajectory points
       points = list(traj.joint_trajectory.points)

       # Cycle through all points and scale the time from start, speed and acceleration
       for i in range(n_points):
           point = JointTrajectoryPoint()
           point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
           point.velocities = list(traj.joint_trajectory.points[i].velocities)
           point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
           point.positions = traj.joint_trajectory.points[i].positions
                         
           for j in range(n_joints):
               point.velocities[j] = point.velocities[j] * scale
               point.accelerations[j] = point.accelerations[j] * scale * scale
            
           points[i] = point

       # Assign the modified points to the new trajectory
       new_traj.joint_trajectory.points = points

       # Return the new trajecotry
       return new_traj
###################################################################################################




moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
rospy.init_node('move_group_python_interface_tutorial', anonymous=True) #initialize the node 

robot = moveit_commander.RobotCommander() #define the robot
scene = moveit_commander.PlanningSceneInterface() #define the scene
group = moveit_commander.MoveGroupCommander("manipulator") #define the planning group (from the moveit packet 'manipulator' planning group)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory) #create a publisher that publishes a plan to the topic: '/move_group/display_planned_path'


#################################MANIPULATION BY JOINT POSITION#######################################################
group_variable_values = group.get_current_joint_values() #create variable that stores joint values

Assign values to joints
group_variable_values[0] = group_variable_values[0]
group_variable_values[1] = group_variable_values[1]
group_variable_values[2] = group_variable_values[2]
group_variable_values[3] = group_variable_values[3]
group_variable_values[4] = group_variable_values[4]
group_variable_values[5] = group_variable_values[5]

group.set_joint_value_target(group_variable_values) #set target joint values for 'manipulator' group
 
plan1 = group.plan() #call plan function to plan the path
rospy.sleep(1)
scaled_traj1 = scale_trajectory_speed(plan1, 0.15) #Scale the trajectory speed
group.execute(scaled_traj1) #execute plan on real/simulation robot
rospy.sleep(1) #sleep 2 seconds
######################################################################################################################


################################manipulation by orientation and position##############################################
pose_target = group.get_current_pose() #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"

#assign values
pose_target.pose.orientation.w = pose_target.pose.orientation.w  
pose_target.pose.orientation.x = pose_target.pose.orientation.x 
pose_target.pose.orientation.y = pose_target.pose.orientation.y  
pose_target.pose.orientation.z = pose_target.pose.orientation.z 
pose_target.pose.position.x = pose_target.pose.position.x 
pose_target.pose.position.y = pose_target.pose.position.y  
pose_target.pose.position.z = pose_target.pose.position.z 

group.set_pose_target(pose_target) #set pose_target as the goal pose of 'manipulator' group 

plan2 = group.plan() #call plan function to plan the path
rospy.sleep(1)
scaled_traj2 = scale_trajectory_speed(plan2, 0.15) #Scale the trajectory speed
group.execute(scaled_traj2) #execute plan on real/simulation robot
rospy.sleep(5) #sleep 5 seconds
######################################################################################################################

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

























