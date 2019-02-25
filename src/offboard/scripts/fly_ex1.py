#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8
#grasp movement test

import rospy 
import roslib
import PID
import time
import tf
import RT
import numpy as np
from apriltags.msg import AprilTagDetections
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget, AttitudeTarget, State, RCIn
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PointStamped, Vector3, Vector3Stamped, TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from offboard.msg import DroneInfo
from offboard.msg import Arm

setpoint_yaw = 0
setpoint_x = setpoint_y = setpoint_z = 0
enable_setpoint = 1
KNOB_L = KNOB_R = BACK_ADJ = 100
SWITCH = 100
detected_tags = 0
feedback_mode = 1

pos_fuse_x = pos_fuse_y = pos_fuse_z = 0
odom_roll = odom_pitch = odom_yaw = 0
InitTko = True
search_direction = 1
TAKEOFF_ENABLE = True

claw_rotation = 0.0 


OUT = False

drop_x = drop_y = 0
drop_z = 0.25
grasp_x_bais = 0.35
grasp_z_bais = -0.4 
target_x = target_y = target_z = 0
target_yaw_bais = 0
Received_det = False
Search = True

dof2_x = dof2_y = 0
dof3_x = dof3_y = 0
grasp_x = 0.35 #末端抓取器的抓取点位置
grasp_y = -0.4

known_grasp_x = -1.6
known_grasp_y = -0.05
known_grasp_z = 0.25

TAG_TkO_X = -2.57
TAG_TkO_Y = -0.11


DOF2_FLOD_X = 0.3237632768798173
DOF2_FLOD_Y = -0.08149061946906042
DOF2_GRASP_X = 0.274768
DOF2_GRASP_Y = -0.43
DOF2_FLYING_X = 0.3237632768798173
DOF2_FLYING_Y = -0.08149061946906042
DOF2_DROP_X = 0.274768
DOF2_DROP_Y = -0.43

L1 = 0.119
L2 = 0.31
# L3 = 0.31 #link2转轴到link3末端抓取点的位置
# L3x = 0.307 #末端抓取点中心在link2坐标系下的表达
# L3y = 0.039
# L4x = 0.185 #相机位置在link2坐标系下的表达
# L4y = 0.075
theta1 = theta2 = theta3 = theta4 = 0
arm_mode = 0
arm_emergency = 0
arm_grasp = 0

set_dof2_x = 0.3
set_dof2_y = -0.3

tko_x = 0
tko_y = 0 
tko_z = 0.4

def callback_rc(rc):
    global SWITCH, KNOB_L, KNOB_R, BACK_ADJ

    if rc.channels[8] == 1065:
        SWITCH = 1 #向上
    if rc.channels[8] == 1933:
        SWITCH = 0 #向下
    if rc.channels[5] > 1065 and rc.channels[5] < 1355:
        KNOB_L = 0
    if rc.channels[5] >= 1355 and rc.channels[5] < 1644:
        KNOB_L = 1
    if rc.channels[5] >= 1644 and rc.channels[5] < 1933:
        KNOB_L = 2

    if rc.channels[7] > 1065 and rc.channels[7] < 1355:
        KNOB_R = 0
    if rc.channels[7] >= 1355 and rc.channels[7] < 1644:
        KNOB_R = 1
    if rc.channels[7] >= 1644 and rc.channels[7] < 1933:
        KNOB_R = 2

    if rc.channels[6] == 1065:
        BACK_ADJ = 1
    else:
        BACK_ADJ = 0
    # print 'SWITCH',SWITCH,'KNOB_L', KNOB_L,'KNOB_R', KNOB_R, 'BACK_ADJ',BACK_ADJ


def callback_tag(tag):
    global detected_tags
    detected_tags = np.shape(tag.detections)[0]

def callback_odom(odom):
    global pos_fuse_x, pos_fuse_y,pos_fuse_z, odom_roll,odom_pitch,odom_yaw, tko_x, tko_y, InitTko

    pos_fuse_x = odom.pose.pose.position.x
    pos_fuse_y = odom.pose.pose.position.y
    pos_fuse_z = odom.pose.pose.position.z
    #开机的时候，开关是在降落状态的
    if SWITCH == 1 and InitTko:
        tko_x = pos_fuse_x
        tko_y = pos_fuse_y
        InitTko = False
    qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    (odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom)
    
def callback_det(det):
    global target_x, target_y, target_z, target_yaw_bais, Received_det
    #相当于乘一个T
    camera_target = np.array([det.pose.position.z, -det.pose.position.x, det.pose.position.y,1])

    camera_target_enu = np.dot(RT.TENU0_matrix(odom_roll, odom_pitch, odom_yaw), camera_target)
    target_x = camera_target_enu[0]
    target_y = camera_target_enu[1]
    target_z = camera_target_enu[2]
    target_yaw_bais = np.arctan2(target_y,target_x)
    # Received_det = True

    # print 'camera',camera_target,'camera_target_enu',camera_target_enu
    # print theta1,theta2,theta3,theta4,arm_roll

def callback_arm(arm):
    global theta1, theta2, theta3, theta4, arm_roll, arm_pitch, dof2_x, dof2_y, dof3_x, dof3_y
    theta1 = arm.theta1/180.0*np.pi
    theta2 = arm.theta2/180.0*np.pi
    theta3 = arm.theta3/180.0*np.pi
    theta4 = arm.theta4/180.0*np.pi
    arm_roll = arm.ArmRoll/180.0*np.pi

    dof2_x, dof2_y = RT.dof2_position(theta1,theta2,L1,L2)
    # print theta1,theta2,dof2_x, dof2_y, L1,L2
    # dof3_x, dof3_y = RT.dof3_position(theta1,theta2,L1,L2, L3x,L3y)


rospy.init_node('drone_info')
rc_in = rospy.Subscriber('mavros/rc/in', RCIn, callback_rc, queue_size=1)
position_tag = rospy.Subscriber('apriltags/detections', AprilTagDetections, callback_tag, queue_size=1)
subodom = rospy.Subscriber('mavros/local_position/odom', Odometry, callback_odom, queue_size=1)
subarm = rospy.Subscriber('arm/theta', Arm, callback_arm)
subdet = rospy.Subscriber('detection', PoseStamped, callback_det)

pub_arm = rospy.Publisher('arm/control', Arm, queue_size=1)
pub_balance = rospy.Publisher('arm/balance', Arm, queue_size=1)
pub_droneinfo =  rospy.Publisher('drone_info', DroneInfo, queue_size=1)

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    dinfo = DroneInfo()

    arm = Arm()

    #Takeoff
    if SWITCH == 0:
        if (pos_fuse_z < tko_z):
            if TAKEOFF_ENABLE: 
                setpoint_x = TAG_TkO_X
                setpoint_y = TAG_TkO_Y
                setpoint_z = tko_z
                TAKEOFF_ENABLE = False
                arm_mode = 1

    if KNOB_R == 2:
        set_dof2_x = 0.40
        set_dof2_y = -0.05
    elif KNOB_R == 1 :
        set_dof2_x = 0.26
        set_dof2_y = -0.08
    elif KNOB_R == 0 :
        set_dof2_x = 0
        set_dof2_y = -0.41

    if KNOB_L == 2:
        arm_grasp = 1
    elif KNOB_L == 1:
        arm_grasp = 0
    elif KNOB_L == 0:
        pass
    # print  arm_mode,KNOB_L
        # arm_grasp = 0
    # setpoint_x = target_x + pos_fuse_x - grasp_x_bais
    # setpoint_y = target_y + pos_fuse_y
    # setpoint_z = target_z - grasp_z_bais

    # setpoint_yaw  = setpoint_yaw + target_yaw_bais

    # LANDING
    if SWITCH == 1:
        setpoint_z = 0
            
    #emergency!!!!        
    if BACK_ADJ == 1:
        # arm_emergency = 1


        claw_rotation = 90
        # if abs(dof2_x - set_dof2_x)<0.02 and abs(dof2_y - set_dof2_y)<0.02:
        #     arm_grasp = 1 
    else:
        # arm_emergency = 0

        claw_rotation = 0
        # if abs(dof2_x - set_dof2_x)<0.02 and abs(dof2_y - set_dof2_y)<0.02:
        #     arm_grasp = 0 

    # set_dof2_x = 0.31
    # set_dof2_y = -0.119
    dinfo.header.stamp = rospy.get_rostime()
    dinfo.setpoint.position.x = setpoint_x
    dinfo.setpoint.position.y = setpoint_y
    dinfo.setpoint.position.z = setpoint_z
    dinfo.setpoint.orientation.w = setpoint_yaw
    dinfo.enable_setpoint = enable_setpoint
    dinfo.fmode = feedback_mode #VINS ro TAG
    pub_droneinfo.publish(dinfo)


    arm.header.stamp = rospy.get_rostime()
    arm.emergency = arm_emergency
    arm.mode = arm_mode
    arm.grasp = arm_grasp
    arm.ClawRotation = claw_rotation 

    arm.dof2point.position.x = set_dof2_x
    arm.dof2point.position.y = set_dof2_y
    pub_arm.publish(arm)
    pub_balance.publish(arm)


    # print 'arm', set_dof2_x,set_dof2_y,'setpoint',setpoint_x,setpoint_y,setpoint_z

    rate.sleep()
