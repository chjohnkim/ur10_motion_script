#!/usr/bin/env python
import sys
import math
import rospy
import copy
import moveit_commander #important
import moveit_msgs.msg #important
import geometry_msgs.msg #important

##___Global Variables___###


##___Initialization___###
moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
rospy.init_node('move_group_python_interface_tutorial', anonymous=True) #initialize the node 

robot = moveit_commander.RobotCommander() #define the robot
scene = moveit_commander.PlanningSceneInterface() #define the scene
group = moveit_commander.MoveGroupCommander("manipulator") #define the planning group (from the moveit packet 'manipulator' planning group)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory) #create a publisher that publishes a plan to the topic: '/move_group/display_planned_path'



def TurnArcAboutAxis(group, axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree, direction, change_orientation):
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    
    print waypoints
    #define the axis of rotation
    if axis is 'x' :
        position_1 = pose_target.position.y
        position_2 = pose_target.position.z
    if axis is 'y' :
        position_1 = pose_target.position.z
        position_2 = pose_target.position.x
    if axis is 'z' :
        position_1 = pose_target.position.x
        position_2 = pose_target.position.y

    circle_radius = ((position_1 - CenterOfCircle_1)**2 + (position_2 - CenterOfCircle_2)**2)**0.5 #Pyth. Theorem
    
    #calculate the angle with respect to 0 degrees based on which quadrant the end_effector is in 
    if position_1 > CenterOfCircle_1 and position_2 > CenterOfCircle_2:
        absolute_angle = math.asin(math.fabs(position_2 - CenterOfCircle_2) / circle_radius)
    if position_1 < CenterOfCircle_1 and position_2 > CenterOfCircle_2:
        absolute_angle = math.pi - math.asin(math.fabs(position_2 - CenterOfCircle_2) / circle_radius)
    if position_1 < CenterOfCircle_1 and position_2 < CenterOfCircle_2:
        absolute_angle = math.pi + math.asin(math.fabs(position_2 - CenterOfCircle_2) / circle_radius)
    if position_1 > CenterOfCircle_1 and position_2 < CenterOfCircle_2:
        absolute_angle = 2.0*math.pi - math.asin(math.fabs(position_2 - CenterOfCircle_2) / circle_radius)
    
    theta = 0 # counter that increases the angle     
    while theta < angle_degree/180.0 * math.pi:
        if axis is 'x' :
            pose_target.position.y = circle_radius * math.cos(theta*direction+absolute_angle)+CenterOfCircle_1 #equation of circle from polar to cartesian x = r*cos(theta)+dx
            pose_target.position.z = circle_radius * math.sin(theta*direction+absolute_angle)+CenterOfCircle_2 #equation of cirlce from polar to cartesian y = r*sin(theta)+dy 
        if axis is 'y' :
            pose_target.position.z = circle_radius * math.cos(theta*direction+absolute_angle)+CenterOfCircle_1
            pose_target.position.x = circle_radius * math.sin(theta*direction+absolute_angle)+CenterOfCircle_2
        if axis is 'z' :
            pose_target.position.x = circle_radius * math.cos(theta*direction+absolute_angle)+CenterOfCircle_1
            pose_target.position.y = circle_radius * math.sin(theta*direction+absolute_angle)+CenterOfCircle_2

        waypoints.append(copy.deepcopy(pose_target))
        theta+=math.pi/360

    #return waypoints
    del waypoints[:2]
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0)
    group.execute(plan3)  
            
###___Retrieve data/status of robot___###
def manipulator_status():
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

###___Manipulate by assigning JOINT VALUES___###
def assign_joint_value():
    group_variable_values = group.get_current_joint_values() #create variable that stores joint values

    #Assign values to joints
    group_variable_values[0] = -1
    group_variable_values[1] = 0
    group_variable_values[2] = 0
    group_variable_values[3] = 0
    group_variable_values[4] = 0
    group_variable_values[5] = 0

    group.set_joint_value_target(group_variable_values) #set target joint values for 'manipulator' group
 
    plan1 = group.plan() #call plan function to plan the path (visualize on rviz)
    group.go(wait=True) #execute plan on real/simulation (gazebo) robot 
    rospy.sleep(2) #sleep 2 seconds


###___Manipulate by assigning POSE TARGET___###
def assign_pose_target():
    pose_target = group.get_current_pose() #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"

    #Assign values
    pose_target.pose.orientation.w = 0.0 
    pose_target.pose.orientation.x = 1.0
    pose_target.pose.orientation.y = 0 
    pose_target.pose.orientation.z = 0
    pose_target.pose.position.x = 1.0 
    pose_target.pose.position.y = 0 
    pose_target.pose.position.z = 0.5

    group.set_pose_target(pose_target) #set pose_target as the goal pose of 'manipulator' group 

    plan2 = group.plan() #call plan function to plan the path
    group.go(wait=True) #execute plan on real/simulation robot
    rospy.sleep(2) #sleep 5 seconds


###___Initiate node; subscribe to topic; call callback function___###
def manipulator_arm_control():
#    assign_pose_target()
#    assign_joint_value()
    #TurnArcAboutAxis(group, 'z', 0.8, 0.0, 360, -1, 'yes')
    TurnArcAboutAxis(group, 'x', -0.1, 0.5, 360, -1, 'yes')
    #TurnArcAboutAxis(group, 'y', 0.4, 0.8, 360, 1, 'yes')
    manipulator_status()
    rospy.spin()


###___MAIN___###
if __name__ == '__main__':

    try:
        manipulator_arm_control()
        
        moveit_commander.roscpp_shutdown() #shut down the moveit_commander

    except rospy.ROSInterruptException: pass
