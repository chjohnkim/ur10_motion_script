#!/usr/bin/env python
import sys
import math
import rospy
import copy
import tf
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

## Turns about a reference center point in path mode or tilt mode 
## User specifies axis:['x'/'y'/'z'], Center of Circle: [y,z / z,x / x,y], Arc turn angle: [degrees], Direction: [1/-1], Tilt Mode: ['yes'/'no'], End_effector tilt axis: ['x'/'y'/'z'], Tilt direction: [1/-1]   
def TurnArcAboutAxis(axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree, direction, tilt, tilt_axis, tilt_direction):
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    resolution = 360 #Calculation of resolution by (180/resolution) degrees 
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

    circle_radius = ((position_1 - CenterOfCircle_1)**2 + (position_2 - CenterOfCircle_2)**2)**0.5 #Pyth. Theorem to find radius
    
    #calculate the global angle with respect to 0 degrees based on which quadrant the end_effector is in 
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
        
        ## Maintain orientation with respect to turning axis  
        if tilt is 'yes':      
            pose_target = TiltAboutAxis(pose_target, resolution, tilt_axis, tilt_direction)

        waypoints.append(copy.deepcopy(pose_target))
       #print waypoints
        theta+=math.pi/resolution # increment counter, defines the number of waypoints 
    del waypoints[:2]
    plan_execute_waypoints(waypoints) 
            
def TiltAboutAxis(pose_target, resolution, tilt_axis, tilt_direction):
    quaternion = (
        pose_target.orientation.x,
        pose_target.orientation.y,
        pose_target.orientation.z,
        pose_target.orientation.w)
            
   # euler = quaternion_to_euler(quaternion[0], quaternion[1], quaternion[2], quaternion[3])     
    euler = tf.transformations.euler_from_quaternion(quaternion) # convert quaternion to euler
    roll = euler[0]
    pitch = euler[1]
    yaw = euler [2]   
    # increment the orientation angle
    if tilt_axis is 'x' :
        roll += tilt_direction*math.pi/resolution
    if tilt_axis is 'y' :
        pitch += tilt_direction*math.pi/resolution
    if tilt_axis is 'z' :
        yaw += tilt_direction*math.pi/resolution
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw) # convert euler to quaternion
    # store values to pose_target
    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]
    return pose_target



###___Manipulate by assigning JOINT VALUES___###
def assign_joint_value(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5):
    group_variable_values = group.get_current_joint_values() #create variable that stores joint values

    #Assign values to joints
    group_variable_values[0] = joint_0
    group_variable_values[1] = joint_1
    group_variable_values[2] = joint_2
    group_variable_values[3] = joint_3
    group_variable_values[4] = joint_4
    group_variable_values[5] = joint_5

    group.set_joint_value_target(group_variable_values) #set target joint values for 'manipulator' group
 
    plan1 = group.plan() #call plan function to plan the path (visualize on rviz)
    group.go(wait=True) #execute plan on real/simulation (gazebo) robot 
    rospy.sleep(2) #sleep 2 seconds


###___Manipulate by assigning POSE TARGET___###
def assign_pose_target(orient_x, orient_y, orient_z, orient_w, pos_x, pos_y, pos_z):
    pose_target = group.get_current_pose() #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"

    #Assign values
    pose_target.pose.orientation.x = orient_x
    pose_target.pose.orientation.y = orient_y
    pose_target.pose.orientation.z = orient_z
    pose_target.pose.orientation.w = orient_w
    pose_target.pose.position.x = pos_x
    pose_target.pose.position.y = pos_y
    pose_target.pose.position.z = pos_z

    group.set_pose_target(pose_target) #set pose_target as the goal pose of 'manipulator' group 

    plan2 = group.plan() #call plan function to plan the path
    group.go(wait=True) #execute plan on real/simulation robot
    rospy.sleep(2) #sleep 5 seconds


def plan_execute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
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

###___Initiate node; subscribe to topic; call callback function___###
def manipulator_arm_control():
    ##Place robot to initial position   
    assign_joint_value(3.14, -1.57, -1.57, 3.14, -1.57, 0)
      
    ##Full Demonstration
    assign_joint_value(0, -1.57, 1.57, -1.57, -1.57, 0)
    assign_pose_target(0, 0, 0.707, 0.707, 0.7, -0.2, 0.4)
    TurnArcAboutAxis('z', 0.7, 0, 45, -1, 'yes', 'z', -1)
    TurnArcAboutAxis('z', 0.7, 0, 45, -1, 'yes', 'z', -1)
    TurnArcAboutAxis('y', 0.4, 0.7, 90, 1, 'yes', 'y', 1) 
#    TurnArcAboutAxis(group, 'x', 0, 0.4, 90, 1, 'yes', 'ee_x', 1)

    ##Demonstration of tilting about y-axis
#    assign_joint_value(0, -1.57, 1.57, -1.57, -1.57, 0)
#    assign_pose_target(1, 0, 0, 0, 0.7, 0, 0.1)
#    TurnArcAboutAxis(group, 'y', 0.1, 0.9, 90, 1, 'yes', 'ee_y', 1)
#    TurnArcAboutAxis(group, 'y', 0.1, 0.9, 90, -1, 'yes', 'ee_y', -1)

    ##Demonstration of tilting about x-axis
#    assign_joint_value(0, -1.57, 1.57, -1.57, -1.57, 0)
#    assign_pose_target(0, 0, 0.707, 0.707, 0.7, -0.2, 0.4)
#    TurnArcAboutAxis(group, 'x', 0, 0.4, 90, -1, 'yes', 'ee_y')
#    TurnArcAboutAxis(group, 'x', 0, 0.4, 90, 1, 'yes', 'ee_y')

    ##Demonstration of tilting about z-axis
#    assign_joint_value(0, -1.57, 1.57, -1.57, -1.57, 0)
#    assign_pose_target(0, 0, 0.707, 0.707, 0.7, -0.2, 0.4)
#    TurnArcAboutAxis(group, 'z', 0.7, 0, 90, -1, 'yes', 'ee_z')
#    TurnArcAboutAxis(group, 'z', 0.7, 0, 90, 1, 'yes', 'ee_z')
  
    manipulator_status()
    rospy.spin()


###___MAIN___###
if __name__ == '__main__':

    try:
        manipulator_arm_control()
        
        moveit_commander.roscpp_shutdown() #shut down the moveit_commander

    except rospy.ROSInterruptException: pass
