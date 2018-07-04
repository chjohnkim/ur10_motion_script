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
from rospy import init_node, is_shutdown
from dynamixel_msgs.msg import JointState
from dynamixel_controllers.srv import *

##___GLOBAL VARIABLES___###
velocity = 0.05 #velocity scaling factor (0, 1.0] - Safe value for a real robot is ~0.05
#Dynamixel
goal_pos = float;
goal_speed = 1.0;

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
dynamixel_pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)



#############################################################################################################################################################################################################
####____DYNAMIXEL CONTROL____####
#############################################################################################################################################################################################################
def dynamixel_increment_speed(increment):
    global goal_speed

    rospy.wait_for_service('/tilt_controller/set_speed')
    try:
        setspeed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed) 
        if increment == 1:
            setspeed(goal_speed + 0.2)
            goal_speed = goal_speed + 0.2
            print (goal_speed)
        if increment == -1:
            setspeed(goal_speed - 0.2)
            goal_speed = goal_speed - 0.2
            print (goal_speed)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e    


###___Set speed of the linear actuator from a range of 0 to 2___###
def dynamixel_set_speed(speed):

    rospy.wait_for_service('/tilt_controller/set_speed')
    try:
        setspeed = rospy.ServiceProxy('/tilt_controller/set_speed', SetSpeed) 
        setspeed(speed)
        print (speed)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e  


###___Set position of the linear actuator where 0 is fully contracted and 255 is fully extended___###
def dynamixel_set_position(position):
    global goal_pos
    
    extend_max = -2;
    contract_max = 1.5;
    goal_pos = position
    #rospy.loginfo('goal_pos: {0}' .format(goal_pos))
    goal_pos = 1.5-(goal_pos/73.0)

    if goal_pos < extend_max:
        goal_pos = extend_max
    if goal_pos > contract_max:
        goal_pos = contract_max
    dynamixel_pub.publish(Float64(goal_pos))
    #rospy.loginfo('goal_pos: {0}' .format(goal_pos))


###___Get command from user to control the gripper (e > extend, c > contract, f > faster, s > slower, 0~255 position)___###
def dynamixel_get_key():
    user_command = raw_input()   
    if user_command in ['e']:
        dynamixel_set_position(255)
        print("Extend")
    elif user_command in ['c']:
        dynamixel_set_position(0)
        print("Contract")
    elif user_command in ['f']:
        dynamixel_increment_speed(1)
        print("Faster")
    elif user_command in ['s']:
        dynamixel_increment_speed(-1)
        print("Slower")
    elif user_command in ['x']:
        dynamixel_set_speed(0)
        print("Emergency Stop")
    else:
        set_pos = int(user_command)
        if set_pos <= 255 and user_command >= 0:
            dynamixel_set_position(set_pos)  
  



#############################################################################################################################################################################################################
####____GRIPPER CONTROL____####
#############################################################################################################################################################################################################
###___Activate gripper___###
def gactive(pub):
  command = outputMsg.CModel_robot_output();
  command.rACT = 1
  command.rGTO = 1
  command.rSP  = 50
  command.rFR  = 150						##force need to be adjusted later
  pub.publish(command)
  rospy.sleep(0.5)
  return command

###___Reset gripper___###
def greset(pub):
  command = outputMsg.CModel_robot_output();
  command.rACT = 0
  pub.publish(command)
  rospy.sleep(0.5)

###___Set position of gripper___###
def gposition(pub,command, position):   ##0=open, 255=close
  #rospy.sleep(0.5)
  command = outputMsg.CModel_robot_output();
  command.rACT = 1
  command.rGTO = 1
  command.rSP  = 50
  command.rFR  = 150						##force need to be adjusted later
  command.rPR = position
  pub.publish(command)
  #rospy.sleep(0.5)
  return command


###___Pick-up Object___###
## This function manipulates gripper and grabs object
## distance is the distance to dive before gripping and velocity is the speed of the motion. It rises 10cm after grabbing object
def pickup(command, distance, vel):
    rospy.sleep(0.5)
    gposition(pub, command, 150) #increment gripper width
    rospy.sleep(1)
    

    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
    x_2 = x_1
    y_2 = y_1
    z_2 = z_1 + distance
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

    command = outputMsg.CModel_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 20
    command.rFR  = 150						##force need to be adjusted later
    command.rPR = 220
    pub.publish(command)
    rospy.sleep(1)

    
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
   
    x_2 = x_1
    y_2 = y_1
    z_2 = z_1 + 0.1
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




#############################################################################################################################################################################################################
####____THIRD-FINGER CONTROL____####
#############################################################################################################################################################################################################
###___FINAL PUSH___###
def final_push():
    regrasp_status = 'pushing'
    regrasp_pub.publish(regrasp_status)
    rospy.sleep(5)


#############################################################################################################################################################################################################
####____SENSING____####
#############################################################################################################################################################################################################


###___FORCE SEEK___###
def force_seek2(axis_world, distance, force_direction, sensitivity, final_offset, vel):
    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
    if axis_world is 'x':
        x_2 = x_1 + distance
        y_2 = y_1
        z_2 = z_1
    if axis_world is 'y':
        x_2 = x_1
        y_2 = y_1 + distance
        z_2 = z_1
    if axis_world is 'z':
        x_2 = x_1
        y_2 = y_1
        z_2 = z_1 + distance
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
    
    forceseek_asyncExecute_waypoints(waypoints)

    rospy.sleep(0.5)
    wrench = rospy.wait_for_message('/robotiq_force_torque_wrench', WrenchStamped, timeout = None)
    
    if force_direction is 'x':
        force_initial = wrench.wrench.force.x
    if force_direction is 'y':
        force_initial = wrench.wrench.force.y
    if force_direction is 'z':
        force_initial = wrench.wrench.force.z
    #print 'threshold = ', math.fabs(force_initial)+sensitivity
    print sensitivity
    force = 0
    i = 0
    while i is not 4:
        wrench = rospy.wait_for_message('/robotiq_force_torque_wrench', WrenchStamped, timeout = None)        
        if force_direction is 'z':
            force = wrench.wrench.force.z
        if force_direction is 'y':
            force = wrench.wrench.force.y
        if force_direction is 'x':
            force = wrench.wrench.force.x 
        #print math.fabs(force) 
        if math.fabs(force) > math.fabs(force_initial)+sensitivity:
            i += 1
        print math.fabs(force) - math.fabs(force_initial)
    print 'STOP'  
    group.stop()
    #relative_pose_target(axis_world, final_offset)
    
    tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
    (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
    x_1 = trans_eelink[0]
    y_1 = trans_eelink[1]
    z_1 = trans_eelink[2]
    if axis_world is 'x':
        x_2 = x_1 + final_offset
        y_2 = y_1
        z_2 = z_1
    if axis_world is 'y':
        x_2 = x_1
        y_2 = y_1 + final_offset
        z_2 = z_1
    if axis_world is 'z':
        x_2 = x_1
        y_2 = y_1
        z_2 = z_1 + final_offset
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
 

###___TRACK APRILTAG___###
##Detect april tag and make the end-effector point directly at the origin of the april tag frame
def point_apriltag(tag_id, tag_frame_name):
    rospy.sleep(1)
    msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout = None) 
    direction_vector = [0,0,0]
    direction_vector_normalized = [0, 0, 0]
    orthogonal_dot = [0, 0, 0]
    detection = False
    x = 0
    while x < 20:
        if msg.detections[x].id is tag_id:
            detection = True
            break
        x += 1
    if detection is True:
    #if msg.detections[0].id is tag_id or msg.detections[1].id is tag_id or msg.detections[2].id is tag_id or msg.detections[3].id is tag_id or msg.detections[4].id is tag_id or msg.detections[5].id is tag_id or msg.detections[6].id is tag_id or msg.detections[7].id is tag_id: 
        rate = rospy.Rate(10)
        tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
        (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
        tf_listener.waitForTransform('/world', tag_frame_name, rospy.Time(), rospy.Duration(4.0))
        (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', tag_frame_name, rospy.Time(0)) #listen to transform between world2tag_0
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
    else:
        print tag_frame_name, ' not found'

###___TRACK APRIL TAG___###
## Detects april tag and moves to the desired position w.r.t. the detected apriltag 
# Specify the values of offset_x, offset_y, offset_z to adjust the final position of the end-effector tip 
def track_apriltag(tag_id, tag_frame_name, offset_x, offset_y, offset_z):
    rospy.sleep(1)
    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
 
    msg = rospy.wait_for_message('/tag_detections', AprilTagDetectionArray, timeout = None)
    detection = False
    x = 0
    while x < 20:
        if msg.detections[x].id is tag_id:
            detection = True
            break
        x += 1
    if detection is True:
        tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
        (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
        tf_listener.waitForTransform('/world', tag_frame_name, rospy.Time(), rospy.Duration(4.0))
        (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', tag_frame_name, rospy.Time(0)) #listen to transform between world2tag_0
        i = 1
        while i is 1:

            pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
            waypoints = []
            waypoints.append(pose_target)

            tf_listener.waitForTransform('/world', '/ee_link', rospy.Time(), rospy.Duration(4.0))
            (trans_eelink, rot_eelink) = tf_listener.lookupTransform('/world', '/ee_link', rospy.Time(0)) #listen to transform between world2ee_link
            tf_listener.waitForTransform('/world', tag_frame_name, rospy.Time(), rospy.Duration(4.0))
            (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', tag_frame_name, rospy.Time(0)) #listen to transform between world2tag_0
        
    
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
            tf_listener.waitForTransform('/world', tag_frame_name, rospy.Time(), rospy.Duration(4.0))
            (trans_tag, rot_tag) = tf_listener.lookupTransform('/world', tag_frame_name, rospy.Time(0)) #listen to transform between world2tag_0
            i = 2
    else:
        print tag_frame_name, ' not found.'  
  


#############################################################################################################################################################################################################
####____RE-GRASP____####
#############################################################################################################################################################################################################

###___REGRASP FUNCTION10 (CARD REGRASP W/ FINGER INTEGRATION)___###
## This regrasp function is modified such that psi can reach all the way to 90 degrees in one go  
def regrasp10(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2-object_width/4, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])
        #Calculate position 
        if theta + psi_current <= 90:
            x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
            z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))
#        elif theta + psi_current is 90:
#            x = PointA_world[0] + (width/2)
#            z = PointA_world[2] + finger_length
        elif theta + psi_current > 90:
            x = PointA_world[0] - math.fabs(finger_length*math.sin(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.cos(math.radians(theta+psi_current-90)))
            z = PointA_world[2] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current-90)))
            
             
        
        #Store Values
        pose_target.position.x = x - object_width*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    y_previous = round(y_initial,0)
    psi_current = 0

    dynamixel_set_speed(2.0)
    phi = 90
    
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    
    while psi_target-1  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = round(math.degrees(y_current), 0)
        if (y_current == y_previous - 1) or (y_current == y_previous + 1):           
            psi_current = psi_current + 1
            y_previous = y_current
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 147.41)/(-0.6783))
        gposition(pub, command, position) #increment gripper width
        
        #Dynamixel Control
        if b < width/2:
            finger_position = finger_length*1000 - opposite - (d/c)*(width/2)
        elif b >= width/2:
            finger_position = finger_length*1000 - a + (a/b)*(b-width/2)
        L = finger_position - 0.0875*1000 - 39.5 #<-------------------------------------------------------------------- probably needs adjustment
        print 'L: ', L
        if L < 149.5 and L >= 145.129:
            phi = 449.275 - 3.00519*L
        elif L < 145.129 and L >= 134.0:
            phi = 211.059 - 1.36471*L
        elif L < 134.0 and L >= 115.814:
            phi = 146.24 - 0.882801*L
        elif L < 115.814 and L >= 61.8861:
            phi = 127.756 - 0.723192*L 
        dynamixel_position = (-43/18)*phi + 235
        dynamixel_set_position(dynamixel_position)
        print opposite
    return [width/1000, opposite/1000]


###___REGRASP FUNCTION 9 (FINGER INTEGRATION)___###  
def regrasp9(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    #print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])
        #Calculate position 
        if theta + psi_current <= 90:
            x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
            z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))
#        elif theta + psi_current is 90:
#            x = PointA_world[0] + (width/2)
#            z = PointA_world[2] + finger_length
        elif theta + psi_current > 90:
            x = PointA_world[0] - math.fabs(finger_length*math.sin(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.cos(math.radians(theta+psi_current-90)))
            z = PointA_world[2] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current-90)))
            
             
        
        #Store Values
        pose_target.position.x = x - (object_width/2)*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    y_previous = round(y_initial,0)
    psi_current = 0
    
    dynamixel_set_speed(2.0)
    phi = 90
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    
    while psi_target-1  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = round(math.degrees(y_current), 0)
        if (y_current == y_previous - 1) or (y_current == y_previous + 1):           
            psi_current = psi_current + 1
            y_previous = y_current
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 147.41)/(-0.6783))
        gposition(pub, command, position) #increment gripper width
        
        #Dynamixel Control
        if b < width/2:
            finger_position = finger_length*1000 - opposite - (d/c)*(width/2)
        elif b >= width/2:
            finger_position = finger_length*1000 - a + (a/b)*(b-width/2)
        L = finger_position - 0.0875*1000 - 36.5 #<-------------------------------------------------------------------- probably needs adjustment
        #print 'L: ', L
        if L < 149.5 and L >= 145.129:
            phi = 449.275 - 3.00519*L
        elif L < 145.129 and L >= 134.0:
            phi = 211.059 - 1.36471*L
        elif L < 134.0 and L >= 115.814:
            phi = 146.24 - 0.882801*L
        elif L < 115.814 and L >= 61.8861:
            phi = 127.756 - 0.723192*L 
        dynamixel_position = (-43/18)*phi + 235
        dynamixel_set_position(dynamixel_position)
                      

        print opposite
    return [width/1000, opposite/1000]





###___REGRASP FUNCTION8 (BATTERY REGRASP)___###
## This regrasp function is modified such that psi can reach all the way to 90 degrees in one go  
def regrasp8(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])
        #Calculate position 
        if theta + psi_current <= 90:
            x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
            z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))
#        elif theta + psi_current is 90:
#            x = PointA_world[0] + (width/2)
#            z = PointA_world[2] + finger_length
        elif theta + psi_current > 90:
            x = PointA_world[0] - math.fabs(finger_length*math.sin(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.cos(math.radians(theta+psi_current-90)))
            z = PointA_world[2] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current-90)))
            
             
        
        #Store Values
        pose_target.position.x = x - (object_width/2)*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    y_previous = round(y_initial,0)
    psi_current = 0
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    
    while psi_target-1  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = round(math.degrees(y_current), 0)
        if (y_current == y_previous - 1) or (y_current == y_previous + 1):           
            psi_current = psi_current + 1
            y_previous = y_current
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 147.41)/(-0.6783))
        gposition(pub, command, position) #increment gripper width
        
        print opposite
    return [width/1000, opposite/1000]



###___REGRASP FUNCTION7 (CARD REGRASP)___###
## This regrasp function is modified such that psi can reach all the way to 90 degrees in one go  
def regrasp7(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2-object_width/4, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])
        #Calculate position 
        if theta + psi_current <= 90:
            x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
            z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))
#        elif theta + psi_current is 90:
#            x = PointA_world[0] + (width/2)
#            z = PointA_world[2] + finger_length
        elif theta + psi_current > 90:
            x = PointA_world[0] - math.fabs(finger_length*math.sin(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.cos(math.radians(theta+psi_current-90)))
            z = PointA_world[2] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current-90))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current-90)))
            
             
        
        #Store Values
        pose_target.position.x = x - object_width*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    y_previous = round(y_initial,0)
    psi_current = 0
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    
    while psi_target-1  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = round(math.degrees(y_current), 0)
        if (y_current == y_previous - 1) or (y_current == y_previous + 1):           
            psi_current = psi_current + 1
            y_previous = y_current
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 147.41)/(-0.6783))
        gposition(pub, command, position) #increment gripper width
        
        print opposite
    return [width/1000, opposite/1000]


###___REGRASP FUNCTION6___###
##This regrasp is the first that worked with accurate geometric interpretation. 
##The logic is calculating the end-effector pose by the following equations:
## ee_link_x = PointA_x + (finger_length)cos(psi+theta) + (width/2)sin(psi+theta)
## ee_link_z = PointA_z + (finger_length) sin(psi+theta) - (width/2)cos(psi+theta)
## Next regrasp will be to simultaneously decrease theta and psi 
def regrasp6(theta, length, psi_target, object_width, axis, direction, tilt_axis, tilt_dierction, command):
    finger_length = 0.2765  ##<----------------------------------------------------------------------------------------------------------------------AROUND 0.280
    pose_target = group.get_current_pose().pose
    pose_position = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
    pose_orientation = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]  
    world2eelink_matrix = tf_listener.fromTranslationRotation(pose_position, pose_orientation) #change base2eelink from transform to matrix
    PointA_eelink = [finger_length, -object_width/2-object_width/4, 0, 1] ##<----------------------------------------------------------------------------TESTING
    PointA_world = numpy.matmul(world2eelink_matrix, PointA_eelink) #Caculate coordinate of point A w.r.t. /world
    
    rpy_initial = group.get_current_rpy()
    
    rpy_initial = [math.degrees(rpy_initial[0]),math.degrees(rpy_initial[1]), math.degrees(rpy_initial[2])]
    #print [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
    print 'initial Pose: ', pose_target
    waypoints = []
    waypoints.append(pose_target)
    psi_current = 0.0
    while psi_current < psi_target: 
        #Calculate width
        a = length * math.cos(math.radians(psi_current))
        b = length * math.sin(math.radians(psi_current))
        c = object_width * math.cos(math.radians(psi_current))
        d = object_width * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        
        #Calculate orientation
        rpy_target = [rpy_initial[0], rpy_initial[1]+psi_current, rpy_initial[2]]
        rpy_target = [math.radians(rpy_target[0]), math.radians(rpy_target[1]), math.radians(rpy_target[2])] 
        quaternion_target = tf.transformations.quaternion_from_euler(rpy_target[0], rpy_target[1], rpy_target[2])

        #Calculate position
        x = PointA_world[0] + math.fabs(finger_length*math.cos(math.radians(theta + psi_current))) + math.fabs((width/2)*math.sin(math.radians(theta+psi_current)))
        z = PointA_world[2] + math.fabs(finger_length*math.sin(math.radians(theta + psi_current))) - math.fabs((width/2)*math.cos(math.radians(theta+psi_current)))
        print x, z
        
        #Store Values
        pose_target.position.x = x - object_width*psi_current/psi_target #<-------------------------------------------------------------------------------TESTING
        pose_target.position.z = z
        pose_target.orientation.x = quaternion_target[0]
        pose_target.orientation.y = quaternion_target[1]
        pose_target.orientation.z = quaternion_target[2]
        pose_target.orientation.w = quaternion_target[3]
        #print psi_current, [pose_target.position.x, pose_target.position.y, pose_target.position.z] 
        waypoints.append(copy.deepcopy(pose_target))
        psi_current += 0.5
    
    del waypoints[0]
    quat_initial = [waypoints[0].orientation.x, waypoints[0].orientation.y, waypoints[0].orientation.z, waypoints[0].orientation.w] 
    euler_initial = tf.transformations.euler_from_quaternion(quat_initial)     
    y_initial = euler_initial[1]
    y_initial = math.degrees(y_initial)
    psi_current = 0
    #del waypoints[:2] 
    regrasp_asyncExecute_waypoints(waypoints)
    #rint waypoints[0]
  
    while psi_target  > psi_current: #while psi is less than the desired psi
        current_pose = group.get_current_pose().pose
        quat_current = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
        euler_current = tf.transformations.euler_from_quaternion(quat_current)  
        y_current = euler_current[1]
        y_current = math.degrees(y_current)
        psi_current = y_current - y_initial
        psi_current = round(psi_current, 0)
        a = length*1000 * math.cos(math.radians(psi_current))
        b = length*1000 * math.sin(math.radians(psi_current))
        c = object_width*1000 * math.cos(math.radians(psi_current))
        d = object_width*1000 * math.sin(math.radians(psi_current))
        opposite = a - d
        width = b + c
        position = int((width - 147.41)/(-0.6783))
        gposition(pub, command, position) #increment gripper width
        #print psi_current
    return [width/1000, opposite/1000]
    
#############################################################################################################################################################################################################
####____MOTION PLAN____####
#############################################################################################################################################################################################################


def TurnArcAboutAxis_2(axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree, direction, tilt, tilt_axis, tilt_direction):
    rospy.sleep(0.5)
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    resolution = 2880 #Calculation of resolution by (180/resolution) degrees 
    #quaternion = [0.5, 0.5, -0.5, 0.5]
    quaternion = [pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w]
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
    
    #print pose_target.orientation
    theta = 0 # counter that increases the angle  
    flag = 1   
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
        
        while flag is 1:     
            euler = tf.transformations.euler_from_quaternion(quaternion) # convert quaternion to euler
            
            roll = euler[0]
            pitch = euler[1]
            yaw = euler [2] 
            flag = 2
        
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

        waypoints.append(copy.deepcopy(pose_target))
        theta+=math.pi/resolution # increment counter, defines the number of waypoints 
    del waypoints[:2]
    plan_execute_waypoints(waypoints) 


def TurnArc_Battery2(offset, axis, angle_degree, direction, tilt_axis, tilt_direction):
    #offset = 0.28 # Offset of the reference turning point with respect x-axis downards from the ee_link origin 
    pose = group.get_current_pose()
    CenterOfCircle_1 = pose.pose.position.z-offset
    CenterOfCircle_2 = pose.pose.position.x+0.00705
    TurnArcAboutAxis_Battery(axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree-1, direction, 'yes', tilt_axis, tilt_direction)
    return [CenterOfCircle_1, CenterOfCircle_2]

###___TurnArcAboutAxis for Battery Inesrtion___###
## This function is made to overcome a singularity point and also for automation of battery insertion routine 
def TurnArc_Battery(offset, axis, angle_degree, direction, tilt_axis, tilt_direction):
    #offset = 0.28 # Offset of the reference turning point with respect x-axis downards from the ee_link origin 
    pose = group.get_current_pose()
    CenterOfCircle_1 = pose.pose.position.z-offset
    CenterOfCircle_2 = pose.pose.position.x
    TurnArcAboutAxis_Battery(axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree-1, direction, 'yes', tilt_axis, tilt_direction)
    return [CenterOfCircle_1, CenterOfCircle_2]

def TurnArcAboutAxis_Battery(axis, CenterOfCircle_1, CenterOfCircle_2, angle_degree, direction, tilt, tilt_axis, tilt_direction):
    rospy.sleep(0.5)
    pose_target = group.get_current_pose().pose #create a pose variable. The parameters can be seen from "$ rosmsg show Pose"
    waypoints = []
    waypoints.append(pose_target)
    resolution = 2880 #Calculation of resolution by (180/resolution) degrees 
    quaternion = [0.5, 0.5, -0.5, 0.5]
  
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
    
    #print pose_target.orientation
    theta = 0 # counter that increases the angle  
    flag = 1   
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
        
        while flag is 1:     
            euler = tf.transformations.euler_from_quaternion(quaternion) # convert quaternion to euler
            
            roll = euler[0]
            pitch = euler[1]
            yaw = euler [2] 
            flag = 2
        
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

        waypoints.append(copy.deepcopy(pose_target))
        theta+=math.pi/resolution # increment counter, defines the number of waypoints 
    del waypoints[:2]
    plan_execute_waypoints(waypoints) 

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
    
    #print pose_target.orientation
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
    #print pose_target.orientation

    #assign_pose_target('nil', 'nil', 'nil', 0.271, 0.653, -0.653, 0.271)   #debug
    
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



###___FORCE SEEK___###
def linear_path(axis_world, distance, vel):
    resolution = 0.05 #resolution is interpreted as 1/resolution = number of interpolated points in the path
    pose_target = group.get_current_pose().pose
    x_1 = pose_target.position.x
    y_1 = pose_target.position.y
    z_1 = pose_target.position.z
    if axis_world is 'x':
        x_2 = x_1 + distance
        y_2 = y_1
        z_2 = z_1
    if axis_world is 'y':
        x_2 = x_1
        y_2 = y_1 + distance
        z_2 = z_1
    if axis_world is 'z':
        x_2 = x_1
        y_2 = y_1
        z_2 = z_1 + distance
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
 
def plan_asyncExecute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, velocity) #parameter that changes velocity
    group.execute(plan, wait = False)

def regrasp_asyncExecute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, 0.02) #parameter that changes velocity
    group.execute(plan, wait = False)

def forceseek_asyncExecute_waypoints(waypoints):
    (plan3, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0) #parameters(waypoints, resolution_1cm, jump_threshold)
    plan= group.retime_trajectory(robot.get_current_state(), plan3, 0.002) #parameter that changes velocity
    group.execute(plan, wait = False)

#############################################################################################################################################################################################################
####____STATUS____####
#############################################################################################################################################################################################################


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

#############################################################################################################################################################################################################
####____MAIN____####
#############################################################################################################################################################################################################
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
#    TurnArcAboutAxis('y', 0.22434, -0.79, 45, 1, 'yes', 'y', -1)
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


##TEMP
#    assign_pose_target(-0.52, 0.1166, 0.22434, 0.0, 0.707, -0.707, 0.0) ## REAL ROBOT ENVIRONMENT
#    TurnArcAboutAxis('y', 0.22434, -0.79, 90, -1, 'yes', 'y', 1)
#    object_length = 0.0853 #object total length in meters
#    object_width = 0.005
#    delta_0 = 0.040 #insertion length in meters
#    theta_0 = 45 #target theta
#    offset = 0.2765 + object_length - delta_0  
#    command = gactive(pub)
#    pivot = TurnArc_Battery(offset, 'y', 90-theta_0, 1, 'y', 1)
#    rospy.sleep(1)
#    [width, opposite] = regrasp7(theta_0, delta_0, 89, object_width, 'y', 1, 'y', 1, command)
#    TurnArcAboutAxis('y', pivot[0], pivot[1], theta_0-5, 1, 'yes', 'y', 1)

##TEMP

###___CARD INSERTION ROUTINE___###
##################################################################################################
###___PICK UP CARD ROUTINE___###
#    object_length = 0.0853 #object total length in meters
#    object_width = 0.005
#    delta_0 = 0.040 #insertion length in meters
#    theta_0 = 30 #target theta
#    offset = 0.2765 + object_length - delta_0  
#    command = gactive(pub)
#    rospy.sleep(0.5) 
#    gposition(pub, command, 220)
#    assign_joint_value(0.025, -2.114, -1.034, -4.706, -1.569, 1.597) # WIDE VIEW POSITION
#    rospy.sleep(1)
#    track_apriltag(6, '/tag_6', -0.018, 0.021, 0.41)
#    rospy.sleep(1)
#    force_seek2('z', -0.1, 'z', 5, 0.003, 0.01)
#    pickup(command, -delta_0, 0.05)


#########___GO TO CARD RECEPTACLE AND INSERTION ROUTINE___###
#    assign_joint_value(0.025, -2.114, -1.034, -4.706, -1.569, 1.597) # WIDE VIEW POSITION    
#    track_apriltag(5, '/tag_5', -0.09, -0.0085, 0.405)
#    rospy.sleep(1)
#    force_seek2('z', -0.1, 'z', 5, 0.001, 0.01)
#    rospy.sleep(1)
#    force_seek2('x', -0.1, 'x', 5, 0.001, 0.01)
#    pivot = TurnArc_Battery(offset, 'y', 90-theta_0, 1, 'y', 1)
#    rospy.sleep(1)
#    [width, opposite] = regrasp7(theta_0, delta_0, 70, object_width, 'y', 1, 'y', 1, command)
#    rospy.sleep(1)
#    TurnArcAboutAxis_2('y', pivot[0], pivot[1], 20, 1, 'yes', 'y', 1)
#    rospy.sleep(0.5)
#    
#    ##PUSHING START##
#    dynamixel_set_speed(2)
#    dynamixel_set_position(160)
#    rospy.sleep(2)
#    dynamixel_set_speed(1.0)
#    dynamixel_set_position(240)
#    rospy.sleep(3)
#    dynamixel_set_position(0)
#    rospy.sleep(2)
#    ##PUSHING END##
#    ##DROPPING START##
##    rospy.sleep(0.5)
##    linear_path('x', 0.010, 0)
##    rospy.sleep(0.5)
##    linear_path('z', 0.020, 0)
##    rospy.sleep(0.5)
#    ##DROPPING END##
#    assign_joint_value(0.025, -2.114, -1.034, -4.706, -1.569, 1.597)
################################################################################################



####___BATTERY INSERTION ROUTINE___###
#################################################################################################
#####___PICK UP BATTERY ROUTINE (TAG_1~4)___###
    object_length = 0.049 #object total length in meters
    object_width = 0.0141
    delta_0 = 0.020 #insertion length in meters
    theta_0 = 40 #target theta
    offset = 0.281 + object_length - delta_0  
    command = gactive(pub)
    rospy.sleep(0.5)
    gposition(pub, command, 220)
    assign_joint_value(0.025, -2.114, -1.034, -4.706, -1.569, 1.597)  #WIDE VIEW POSITION
    track_apriltag(3, '/tag_3', -0.003, 0.012, 0.37)
    rospy.sleep(1)
    force_seek2('z', -0.1, 'z', 5, 0.003, 0.01)
    pickup(command, -delta_0+0.006, 0.05)

#######___BATTERY INSERTION ROUTINE (WITH FEEDBACK)___###
    assign_joint_value(0.025, -2.114, -1.034, -4.706, -1.569, 1.597) # WIDE VIEW POSITION
    track_apriltag(7, '/tag_7', -0.027, 0.010, 0.33)
    rospy.sleep(1)
    force_seek2('z', -0.1, 'z', 3, 0.001, 0.002)
    rospy.sleep(1)

    
    linear_path('x', -0.005, 0)
    rospy.sleep(1)

##    force_seek2('x', -0.1, 'x', 3, 0.000, 0.002)
    pivot = TurnArc_Battery2(offset, 'y', 90 - theta_0, 1, 'y', 1)
    rospy.sleep(1)
    [width, opposite] = regrasp9(theta_0, delta_0, 50, object_width, 'y', 1, 'y', 1, command)
    rospy.sleep(1)
    TurnArcAboutAxis_2('y', pivot[0], pivot[1], 10, 1, 'yes', 'y', 1)
    dynamixel_set_speed(2)
    dynamixel_set_position(0)
    
#################################################################################################



###SIMULATION
#    assign_pose_target(-0.52, 0.1166, 0.22434, 0.0, 0.707, -0.707, 0.0) # REAL ROBOT ENVIRONMENT
#    TurnArcAboutAxis('y', 0.22434, -0.79, 90, -1, 'yes', 'y', 1)
#    object_length = 0.049 #object total length in meters
#    object_width = 0.0141
#    delta_0 = 0.015 #insertion length in meters
#    theta_0 = 40 #target theta
#    offset = 0.2765 + object_length - delta_0  
#    command = gactive(pub)
#    pivot = TurnArc_Battery2(offset, 'y', 90 - theta_0, 1, 'y', 1)
#    [width, opposite] = regrasp9(theta_0, delta_0, 40, object_width, 'y', 1, 'y', 1, command)

 

 

#    manipulator_status() #debug
    rospy.spin()



###___MAIN___###
if __name__ == '__main__':

    try:
        
        manipulator_arm_control()
        
        moveit_commander.roscpp_shutdown() #shut down the moveit_commander

    except rospy.ROSInterruptException: pass
