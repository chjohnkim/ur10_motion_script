#!/usr/bin/env python
import sys
import math
import rospy
import copy
import tf
import numpy
import moveit_commander 
import moveit_msgs.msg
import std_msgs.msg 
import geometry_msgs.msg 
import roslib; roslib.load_manifest('robotiq_c_model_control')
from robotiq_c_model_control.msg import _CModel_robot_output as outputMsg
from apriltags_ros.msg import * 
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import *

##___GLOBAL VARIABLES___###
velocity = 0.05 #velocity scaling factor (0, 1.0] - Safe value for a real robot is ~0.05

##___INITIALIZATION___###
moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
rospy.init_node('move_group_python_interface_tutorial', anonymous=True) #initialize the node 
robot = moveit_commander.RobotCommander() #define the robot
scene = moveit_commander.PlanningSceneInterface() #define the scene
group = moveit_commander.MoveGroupCommander("manipulator") #define the planning group (from the moveit packet 'manipulator' planning group)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory) #publisher that publishes a plan to the topic
pub = rospy.Publisher('CModelRobotOutput', outputMsg.CModel_robot_output)
regrasp_pub = rospy.Publisher('regrasp_status', String, queue_size = 10)
psi_pub = rospy.Publisher('psi_current', Float32, queue_size = 10)
length_pub = rospy.Publisher('length_value', Float32, queue_size = 10)
tf_listener = tf.TransformListener()
tf_broadcaster = tf.TransformBroadcaster()

 


###___FORCE TORQUE CONTROL___###
## Get feedback from force-torque wrist to move linearly until a certain force is sensed
# axis: axis w.r.t. world frame to move linearly about ('x'/'y'/'z')
# direction: direction of movement: (1/-1)
# final_offset: distance to move back after sensing threshold force (unit: m)
# sensitivity: force difference to detect to stop movement (unit: N)
def ft_sensor_reading(axis, direction, final_offset, sensitivity):
    wrench = rospy.wait_for_message('/robotiq_force_torque_wrench', WrenchStamped, timeout = None)
    if axis is 'z':
        force_initial = wrench.wrench.force.z
    if axis is 'y':
        force_initial = wrench.wrench.force.y
    if axis is 'x':
        force_initial = wrench.wrench.force.x
    force = 0
    while math.fabs(force) < math.fabs(force_initial)+sensitivity:
        wrench = rospy.wait_for_message('/robotiq_force_torque_wrench', WrenchStamped, timeout = None)        
        if axis is 'z':
            force = wrench.wrench.force.z
        if axis is 'y':
            force = wrench.wrench.force.y
        if axis is 'x':
            force = wrench.wrench.force.x     
        relative_pose_target(axis, 0.001*direction)
    relative_pose_target(axis, final_offset*-0.001*direction)

###___GRIPPER CONTROL___###
## Activate gripper
def gactive(pub):
  command = outputMsg.CModel_robot_output();
  command.rACT = 1
  command.rGTO = 1
  command.rSP  = 50
  command.rFR  = 150						##force need to be adjusted later
  pub.publish(command)
  return command

## Rest gripper
def greset(pub):
  command = outputMsg.CModel_robot_output();
  command.rACT = 0
  pub.publish(command)

## Set position of gripper
def gposition(pub,command, position):   ##0=open, 255=close
  command = outputMsg.CModel_robot_output();
  command.rACT = 1
  command.rGTO = 1
  command.rSP  = 50
  command.rFR  = 150						##force need to be adjusted later
  command.rPR = position
  pub.publish(command)
  return command

def final_push():
    regrasp_status = 'pushing'
    regrasp_pub.publish(regrasp_status)
    rospy.sleep(5)

######################################ALGORITHM###################################################################
#   if apriltag_tf is detected:
#    Find x-y-z position of the april tag with respect to the ee_link
#        while ee_link is not directly above the apriltag_tf (set resolution) 
#            Calculate the desired orientation of the ee_link to look directly at the apriltag
#            Rotate to the desired orientation 
#            Increment towards the desired position of the ee_link (1)
#            Simultaneously change orientation such that it continues to stare at the april tag (2)
#            Calculation of Parts (1) and (2) are stored in waypoints()
#            However, only a certain percentage (loop_count * 5%) of the path will be followed 
#            loop count += 1
#        if ee_link is directly staring at and above the april tag
#            go down until the force sensor senses vertical pushing force from the battery receptacle  
#            perform the turn_arc_function with respect to the tip of the battery
#            go forward and push against the spring to feel the perfect amount of force to begin regrasp
             
#(1) This part requires calculating the straight line cartesian path in 3-D to store in waypoints.
#(2) This part requires calculating the oreintation change while moving in that specific straight line cartesian path
######################################ALGORITHM###################################################################


###___TRACK APRILTAG___###
##Detect april tag and make the end-effector point directly at the origin of the april tag frame
def point_apriltag(tag_id):
    msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout = None) 
    direction_vector = [0,0,0]
    direction_vector_normalized = [0, 0, 0]
    orthogonal_dot = [0, 0, 0]
    
    if msg.detections[0].id is tag_id:
        rate = rospy.Rate(10)
        tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
        (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
        tf_listener.waitForTransform('/world', '/tag_0', rospy.Time(), rospy.Duration(4.0))
        (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', '/tag_0', rospy.Time(0)) #listen to transform between world2tag_0
        direction_vector[0] = trans_tag[0] - trans_eelink[0]
        direction_vector[1] = trans_tag[1] - trans_eelink[1]
        direction_vector[2] = trans_tag[2] - trans_eelink[2]
        
        #normalize direction_vector to unit length
        length = math.sqrt(direction_vector[0]*direction_vector[0]+direction_vector[1]*direction_vector[1]+direction_vector[2]*direction_vector[2])
        direction_vector_normalized[0] = direction_vector[0] / length
        direction_vector_normalized[1] = direction_vector[1] / length
        direction_vector_normalized[2] = direction_vector[2] / length
        

        #orthgonal by cross product with a standard vector e_y
        e_y = [0, 1, 0] # this parameter needs to be changed according to the general workspace of the robot 
        orthogonal_standard = numpy.cross(direction_vector_normalized, e_y)
        length = math.sqrt(orthogonal_standard[0]*orthogonal_standard[0]+orthogonal_standard[1]*orthogonal_standard[1]+orthogonal_standard[2]*orthogonal_standard[2])
        orthogonal_standard[0] = orthogonal_standard[0] / length
        orthogonal_standard[1] = orthogonal_standard[1] / length
        orthogonal_standard[2] = orthogonal_standard[2] / length 
        
        #orthogonal by cross product
        orthogonal_cross = numpy.cross(direction_vector_normalized, orthogonal_standard)
 
        #Fill the Rotation matrix 
        I = tf.transformations.identity_matrix()
        I[0,0] = direction_vector_normalized[0]
        I[1,0] = direction_vector_normalized[1]
        I[2,0] = direction_vector_normalized[2]
        I[0,1] = orthogonal_standard[0]
        I[1,1] = orthogonal_standard[1]
        I[2,1] = orthogonal_standard[2]
        I[0,2] = orthogonal_cross[0]
        I[1,2] = orthogonal_cross[1]
        I[2,2] = orthogonal_cross[2]
        I[0,3] = trans_eelink[0]
        I[1,3] = trans_eelink[1]
        I[2,3] = trans_eelink[2]
        quat_from_mat = tf.transformations.quaternion_from_matrix(I)
        assign_pose_target(trans_eelink[0], trans_eelink[1], trans_eelink[2], quat_from_mat[0], quat_from_mat[1], quat_from_mat[2], quat_from_mat[3])

###___TRACK APRIL TAG___###
## Detects april tag and moves to the desired position w.r.t. the detected apriltag 
# Specify the values of offset_x, offset_y, offset_z to adjust the final position of the end-effector tip 
def track_apriltag(tag_id):
    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
    
    ###ADJUST THESE PARAMETERS TO MAKE UP FOR THE CAMERA CALIBRATION###
    offset_x = 0.03  
    offset_y = 0.022
    offset_z = 0.34 # offset of height from the goal position 
    
    msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout = None)
    if msg.detections[0].id is tag_id:

        tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
        (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
        tf_listener.waitForTransform('/world', '/tag_0', rospy.Time(), rospy.Duration(4.0))
        (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', '/tag_0', rospy.Time(0)) #listen to transform between world2tag_0
        i = 1
        while i is 1:

            pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
            waypoints = []
            waypoints.append(pose_target)

            tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
            (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
            tf_listener.waitForTransform('/world', '/tag_0', rospy.Time(), rospy.Duration(4.0))
            (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', '/tag_0', rospy.Time(0)) #listen to transform between world2tag_0
        
    
            x_1 = trans_eelink[0]
            y_1 = trans_eelink[1]
            z_1 = trans_eelink[2]
            x_2 = trans_tag[0]+offset_x     
            y_2 = trans_tag[1]+offset_y 
            z_2 = trans_tag[2] + offset_z
            v = [x_2-x_1, y_2-y_1, z_2-z_1]
            
            t = 0 # counter/increasing variabe for the parametric equation of straight line      
            while t <= 1.01:
                pose_target.position.x = x_1 + v[0]*t
                pose_target.position.y = y_1 + v[1]*t
                pose_target.position.z = z_1 + v[2]*t    
                store_x = x_1 + v[0]*t
                store_y = y_1 + v[1]*t
                store_z = z_1 + v[2]*t
   
                direction_vector = [0,0,0]
                direction_vector_normalized = [0, 0, 0]
                orthogonal_dot = [0, 0, 0]
                    
                direction_vector[0] = x_2 - store_x 
                direction_vector[1] = y_2 - store_y 
                direction_vector[2] = z_2 - store_z - offset_z
        
                #normalize direction_vector to unit length
                length = math.sqrt(direction_vector[0]*direction_vector[0]+direction_vector[1]*direction_vector[1]+direction_vector[2]*direction_vector[2])
                direction_vector_normalized[0] = direction_vector[0] / length
                direction_vector_normalized[1] = direction_vector[1] / length
                direction_vector_normalized[2] = direction_vector[2] / length
        
                #orthgonal by cross product with a standard vector e_y
                e_y = [0, 1, 0] # this parameter needs to be changed according to the general workspace of the robot 
                orthogonal_standard = numpy.cross(direction_vector_normalized, e_y)
                length = math.sqrt(orthogonal_standard[0]*orthogonal_standard[0]+orthogonal_standard[1]*orthogonal_standard[1]+orthogonal_standard[2]*orthogonal_standard[2])
                orthogonal_standard[0] = orthogonal_standard[0] / length
                orthogonal_standard[1] = orthogonal_standard[1] / length
                orthogonal_standard[2] = orthogonal_standard[2] / length 
        
                #orthogonal by cross product
                orthogonal_cross = numpy.cross(direction_vector_normalized, orthogonal_standard)

                #Fill the Rotation matrix 
                I = tf.transformations.identity_matrix()
                I[0,0] = direction_vector_normalized[0]
                I[1,0] = direction_vector_normalized[1]
                I[2,0] = direction_vector_normalized[2]
                I[0,1] = orthogonal_standard[0]
                I[1,1] = orthogonal_standard[1]    
                I[2,1] = orthogonal_standard[2]    
                I[0,2] = orthogonal_cross[0]
                I[1,2] = orthogonal_cross[1]
                I[2,2] = orthogonal_cross[2]
                I[0,3] = store_x    
                I[1,3] = store_y
                I[2,3] = store_z
                quat_from_mat = tf.transformations.quaternion_from_matrix(I)    
                
                pose_target.orientation.x = quat_from_mat[0]
                pose_target.orientation.y = quat_from_mat[1]
                pose_target.orientation.z = quat_from_mat[2]
                pose_target.orientation.w = quat_from_mat[3]
                waypoints.append(copy.deepcopy(pose_target))
                 
                t += resolution 
    
            del waypoints[:1]
            plan_execute_waypoints(waypoints)
        
            tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
            (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
            tf_listener.waitForTransform('/world', '/tag_0', rospy.Time(), rospy.Duration(4.0))
            (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', '/tag_0', rospy.Time(0)) #listen to transform between world2tag_0
            i = 2

###___LINEAR PATH PLAN FROM TWO POINTS___###
##This function takes two points in 3D coordinate wrt world frame and plans a linear path for the end_effector to move to
##The first point is the current position of the end-effector while the second point is the desired position
def two_points_linear_path():
    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
    tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
    (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
    tf_listener.waitForTransform('/world', '/tag_0', rospy.Time(), rospy.Duration(4.0))
    (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', '/tag_0', rospy.Time(0)) #listen to transform between world2tag_0
    x_1 = trans_eelink[0]
    y_1 = trans_eelink[1]
    z_1 = trans_eelink[2]
    x_2 = trans_tag[0]
    y_2 = trans_tag[1]
    z_2 = trans_tag[2]
    direction_vector = [x_2-x_1, y_2-y_1, z_2-z_1]
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    t = 0 # counter/increasing variabe for the parametric equation of straight line      
    while t <= 1.01:
        pose_target.position.x = x_1 + direction_vector[0]*t
        pose_target.position.y = y_1 + direction_vector[1]*t
        pose_target.position.z = z_1 + direction_vector[2]*t
        t += resolution 
        
        waypoints.append(copy.deepcopy(pose_target))
         
    del waypoints[:1]
    plan_execute_waypoints(waypoints)

###___TurnArcAboutAxis for Battery Inesrtion___###
## This function is made to overcome a singularity point and also for automation of battery insertion routine 
def TurnArc_Battery(axis, angle_degree, direction, tilt_axis, tilt_direction):
    offset = 0.3 # Offset of the reference turning point with respect x-axis downards from the ee_link origin 
    pose = group.get_current_pose()
    CenterOfCircle_1 = pose.pose.position.z-offset
    CenterOfCircle_2 = pose.pose.position.x
    TurnArcAboutAxis(axis, CenterOfCircle_1, CenterOfCircle_2, 1, direction, 'no', tilt_axis, 1)
    assign_pose_target('nil', 'nil', 'nil', 0.496, 0.504, -0.504, 0.496)
    TurnArcAboutAxis(axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree-1, direction, 'yes', tilt_axis, tilt_direction)


###___REGRASP FUNCTION2___###
## Assuming width =/= 0 for not very thin objects like battery
## Regrasp thin object by simultaneously tiliting end-effector and widening grip (unit: mm)
def regrasp2(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_direction, command): # Assumption is that initial conditions are psi = 0 and opposite = length
    resol = 0.5 # set resolution of incremental movements with respect to psi (unit: degrees)
    rate_hz = 10 # set speed of regrasp by setting update frequency (hz)
    psi_current = 0.0
    i = 1
    while psi_current < psi_target: 
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        center_of_rotation = get_instantaneous_center(opposite, rate_hz)
        position = int((width - 146.17)/(-0.6584))
        psi_current = psi_current + resol
        gposition(pub, command, position) #increment gripper width   
        if axis is 'x':
            TurnArcAboutAxis('x', center_of_rotation[1], center_of_rotation[2], resol, direction, 'yes', tilt_axis, tilt_direction)       
        if axis is 'y':
            TurnArcAboutAxis('y', center_of_rotation[2], center_of_rotation[0], resol, direction, 'yes', tilt_axis, tilt_direction)       
        if axis is 'z':
            TurnArcAboutAxis('z', center_of_rotation[0], center_of_rotation[1], resol, direction, 'yes', tilt_axis, tilt_direction)       
 
        regrasp_status = 'regrasping'
        regrasp_pub.publish(regrasp_status)
        psi_pub.publish(psi_current)
        length_pub.publish(length)
    regrasp_status = '0'
    regrasp_pub.publish(regrasp_status)        

###___REGRASP FUNCTION1___###
## Assuming width == 0 for very thin objects like cards
## Regrasp thin object by simultaneously tiliting end-effector and widening grip (unit: mm)
def regrasp1(theta, length, psi_target, axis, direction, tilt_axis, tilt_direction, command): # Assumption is that initial conditions are psi = 0 and opposite = length
    
    resol = 0.5 # set resolution of incremental movements with respect to psi (unit: degrees)
    rate_hz = 10 # set speed of regrasp by setting update frequency (hz)
    psi_current = 0.0
    i = 1
    while psi_current < psi_target: 
        
        opposite = length * math.sin(math.radians(90-psi_current))

        center_of_rotation = get_instantaneous_center(opposite, rate_hz)

        width = opposite / math.tan(math.radians(90-psi_current+1))
        position = int((width - 130.17)/(-0.6584)) # Gripper position from a range of (0-255)
        psi_current = psi_current + resol
        i += 1 
        gposition(pub, command, position) #increment gripper width   
        if axis is 'x':
            TurnArcAboutAxis('x', center_of_rotation[1], center_of_rotation[2], resol, direction, 'yes', tilt_axis, tilt_direction)       
        if axis is 'y':
            TurnArcAboutAxis('y', center_of_rotation[2], center_of_rotation[0], resol, direction, 'yes', tilt_axis, tilt_direction)       
        if axis is 'z':
            TurnArcAboutAxis('z', center_of_rotation[0], center_of_rotation[1], resol, direction, 'yes', tilt_axis, tilt_direction)       
        #print 'Position: ', position, ' CoR: ', center_of_rotation #' psi_current: ', psi_current, ' width: ', width, ' opposite: ', opposite #debug
        #print width

## Get instantaneous center of rotation for regrasp() function
def get_instantaneous_center(opposite, rate_hz):
    rate = rospy.Rate(rate_hz)       
    displacement = 0.290-(opposite/2)/1000 ##Center of rotation parameter - WHY 290???? Probably from measurement 
    #print 'displacement: ', displacement
    tf_listener.waitForTransform('/base_link', '/ee_link', rospy.Time(), rospy.Duration(4.0))
    (trans1, rot1) = tf_listener.lookupTransform('/base_link', '/ee_link', rospy.Time(0)) #listen to transform between base_link2ee_link
    base2eelink_matrix = tf_listener.fromTranslationRotation(trans1, rot1) #change base2eelink from transform to matrix
    eelink2eetip_matrix = tf_listener.fromTranslationRotation((displacement, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)) #change eelink2eetip from transform to matrix
    base2eetip_matrix = numpy.matmul(base2eelink_matrix, eelink2eetip_matrix) #combine transformation: base2eetip = base2eelink x eelink2eetip
    scale, shear, rpy_angles, translation_vector, perspective = tf.transformations.decompose_matrix(base2eetip_matrix) #change base2eetip from matrix to transform
    quaternion = tf.transformations.quaternion_from_euler(rpy_angles[0], rpy_angles[1], rpy_angles[2])
    rate.sleep()
    return translation_vector


###___TURN ARC FUNCTION___###
## Turns about a reference center point in path mode or tilt mode 
## User specifies axis:['x'/'y'/'z'], Center of Circle: [y,z / z,x / x,y], Arc turn angle: [degrees], Direction: [1/-1], Tilt Mode: ['yes'/'no'], End_effector tilt axis: ['x'/'y'/'z'], Tilt direction: [1/-1]   
def TurnArcAboutAxis(axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree, direction, tilt, tilt_axis, tilt_direction):
    rospy.sleep(0.5)
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    resolution = 2880 #Calculation of resolution by (180/resolution) degrees 
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
    #print quaternion, '          ' , roll, pitch, yaw #debug
    # store values to pose_target
    pose_target.orientation.x = quaternion[0]
    pose_target.orientation.y = quaternion[1]
    pose_target.orientation.z = quaternion[2]
    pose_target.orientation.w = quaternion[3]
    return pose_target


###___JOINT VALUE MANIPULATION___###
## Manipulate by assigning joint values
def assign_joint_value(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5):
    group.set_max_velocity_scaling_factor(velocity)
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
    #rospy.sleep(2) #sleep 2 seconds

###___POSE TARGET MANIPULATION___###
## Manipulate by assigning pose target
def assign_pose_target(pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w):
    group.set_max_velocity_scaling_factor(velocity)
    pose_target = group.get_current_pose() #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"

    #Assign values
    if pos_x is 'nil':
        pass
    else:     
        pose_target.pose.position.x = pos_x
    if pos_y is 'nil':
        pass
    else:    
        pose_target.pose.position.y = pos_y
    if pos_z is 'nil':
        pass
    else:    
        pose_target.pose.position.z = pos_z
    if orient_x is 'nil':
        pass
    else: 
        pose_target.pose.orientation.x = orient_x
    if orient_y is 'nil':
        pass
    else: 
        pose_target.pose.orientation.y = orient_y
    if orient_z is 'nil':
        pass
    else: 
        pose_target.pose.orientation.z = orient_z
    if orient_w is 'nil':
        pass
    else:     
        pose_target.pose.orientation.w = orient_w
    
    group.set_pose_target(pose_target) #set pose_target as the goal pose of 'manipulator' group 

    plan2 = group.plan() #call plan function to plan the path
    group.go(wait=True) #execute plan on real/simulation robot
    #rospy.sleep(2) #sleep 5 seconds

###___RELATIVE JOINT VALUE MANIPULATION___###
## Manipulate by assigning relative joint values w.r.t. current joint values of robot
def relative_joint_value(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5):
    group.set_max_velocity_scaling_factor(velocity)
    group_variable_values = group.get_current_joint_values() #create variable that stores joint values

    #Assign values to joints
    group_variable_values[0] += joint_0
    group_variable_values[1] += joint_1
    group_variable_values[2] += joint_2
    group_variable_values[3] += joint_3
    group_variable_values[4] += joint_4
    group_variable_values[5] += joint_5

    group.set_joint_value_target(group_variable_values) #set target joint values for 'manipulator' group
 
    plan1 = group.plan() #call plan function to plan the path (visualize on rviz)
    group.go(wait=True) #execute plan on real/simulation (gazebo) robot 
    #rospy.sleep(2) #sleep 2 seconds

###___RELATIVE POSE TARGET MANIPULATION___###
## Manipulate by moving gripper linearly with respect to world frame
def relative_pose_target(axis_world, distance):
    group.set_max_velocity_scaling_factor(velocity)
    pose_target = group.get_current_pose() #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    if axis_world is 'x':
        pose_target.pose.position.x += distance
    if axis_world is 'y':
        pose_target.pose.position.y += distance
    if axis_world is 'z':
        pose_target.pose.position.z += distance
    group.set_pose_target(pose_target) #set pose_target as the goal pose of 'manipulator' group 

    plan2 = group.plan() #call plan function to plan the path
    group.go(wait=True) #execute plan on real/simulation robot
   
def plan_execute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, velocity) #parameter that changes velocity
    group.execute(plan) 


###___STATUS ROBOT___###
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


###____MAIN____####
###___Initiate node; subscribe to topic; call callback function___###
def manipulator_arm_control():

###___LIST OF FUNCTIONS___###
##      assign_pose_target(pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w)
##      assign_joint_value(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
##      relative_pose_target(axis_world, distance)
##      relative_joint_value(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
##      TurnArcAboutAxis(axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree, direction, tilt, tilt_axis, tilt_direction)
##      regrasp(theta, length, psi_target, axis, direction, tilt_axis, tilt_direction)


###___TEMP___###    
    
   

###___MOTION PLAN TO SET ROBOT TO REAL ENVIRONMNET for GAZEBO___###
#    relative_joint_value(0, -math.pi/2, 0, 0, 0, 0)
#    relative_joint_value(0, 0, -3*math.pi/4, 0, 0, 0)
#    relative_joint_value(0, 0, 0, -3*math.pi/4, 0, 0)
#    relative_joint_value(0, 0, 0, 0, -math.pi/2, 0)
#    assign_pose_target(-0.52, 0.1166, 0.22434, 0.0, 0.707, -0.707, 0.0) # REAL ROBOT ENVIRONMENT    


###___TURNARC DEMO___### 
#    assign_pose_target(-0.52, 0.1166, 0.22434, 0.0, 0.707, -0.707, 0.0) ## REAL ROBOT ENVIRONMENT
#    TurnArcAboutAxis('y', 0.22434, -0.79, 90, -1, 'yes', 'y', 1)
#    TurnArcAboutAxis('y', 0.22434, -0.79, 90, 1, 'yes', 'y', -1)
#    TurnArcAboutAxis('y', 0.22434, -0.79, 70, -1, 'yes', 'y', 1)
#    TurnArcAboutAxis('y', 0.22434, -0.79, 70, 1, 'yes', 'y', -1)


###___BATTERY INSERTION DEMO1___###    
#    command = gactive(pub) 
#    gposition(pub, command, 200)
#    assign_pose_target(-0.51686279, 0.134689, 0.1892067, 0.0, -0.7071, 0.7071, 0.0)
#    relative_pose_target('x', -0.085)
#    relative_pose_target('z', -0.10)    
#    TurnArcAboutAxis('y', 0.0888, -0.82, 45, -1, 'yes', 'y', 1)
#    relative_pose_target('z', -0.02)
#    regrasp1(45.0, 15, 20, 'y', -1, 'y', 1, command)
#    relative_pose_target('x', -0.006)
    

###___BATTERY INSERTION DEMO2___###    
#    command = gactive(pub) 
#    gposition(pub, command, 200)
#    assign_pose_target(-0.51686279, 0.134689, 0.1892067, 0.0, -0.7071, 0.7071, 0.0)
#    relative_pose_target('x', -0.085)
#    relative_pose_target('z', -0.10)    
#    TurnArcAboutAxis('y', 0.0888, -0.82, 45, -1, 'yes', 'y', 1)
#    relative_pose_target('z', -0.018)
#    regrasp2(45.0, 15, 20, 14, 'y', -1, 'y', 1, command)
#    relative_pose_target('x', -0.005)
#    final_push()

###___BATTERY INSERTION ROUTINE (WITH FEEDBACK)___###
    assign_joint_value(0.105, -1.209, -1.692, -4.441, -1.415, 2.039) # WIDE VIEW POSITION
    point_apriltag(0)
    track_apriltag(0)
#    ft_sensor_reading('z', -1, 2, 3)
##    relative_joint_value(0, 0, 0, 0, 0, math.pi/2)
##    ft_sensor_reading('y', -1, 1)
##    relative_joint_value(0, 0, 0, 0, 0, -math.pi/2)
#    ft_sensor_reading('x', -1, 5, 3)
    TurnArc_Battery('y', 45, 1, 'y', -1)



#    manipulator_status() #debug
    rospy.spin()







###___MAIN___###
if __name__ == '__main__':

    try:
        
        manipulator_arm_control()
        
        moveit_commander.roscpp_shutdown() #shut down the moveit_commander

    except rospy.ROSInterruptException: pass
