#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8
#主流程控制器

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

setpoint_yaw = 0
setpoint_x = setpoint_y = setpoint_z = 0
enable_setpoint = 1
KNOB_L = KNOB_R = BACK_ADJ = 100
SWITCH = 100
detected_tags = 0
feedback_mode = 0
tko_x = tko_y = 0
tko_z = 0.1
tko_yaw = 0

centric_x = 0 
centric_y = 0
centric_set = 0
SET_ONCE = 0

vaild_angle = centric_distance = TUNNEL_VAILD = 0
waypoint_yaw = 0
TUNNING_ENABLE = 0
TUNNING_FINISHED = 1

YAW_ADJ_TOR = 0.1 #if new angle is not bigger than this value, the drone will not adjust itself.
MAX_FLY_RANGE = 5

waypoint_x = waypoint_y = 0

pos_fuse_x = pos_fuse_y = pos_fuse_z = 0
odom_roll = odom_pitch = odom_yaw = 0
InitTko = True
search_direction = 1
TAKEOFF_ENABLE = True

OUT = False
AUTO_ENABLE = False
TAG_TkO_X = -2.56
TAG_TkO_Y = -0.08


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

    if rc.channels[6] <= 1100:
        BACK_ADJ = 1
    else:
        BACK_ADJ = 0
    # print 'SWITCH',SWITCH,'KNOB_L', KNOB_L,'KNOB_R', KNOB_R, 'BACK_ADJ',BACK_ADJ

def l2_distance(x1,y1,x2,y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def callback_tag(tag):
    global detected_tags
    detected_tags = np.shape(tag.detections)[0]

def callback_odom(odom):
    global pos_fuse_x, pos_fuse_y,pos_fuse_z, odom_roll,odom_pitch,odom_yaw, tko_x, tko_y, InitTko, tko_yaw

    pos_fuse_x = odom.pose.pose.position.x
    pos_fuse_y = odom.pose.pose.position.y
    pos_fuse_z = odom.pose.pose.position.z
    qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    (odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom)
    #开机的时候，开关是在降落状态的
    if SWITCH == 1 and InitTko:
        tko_x = pos_fuse_x
        tko_y = pos_fuse_y
        tko_yaw = odom_yaw
        InitTko = False

def callback_tunnel(tunnel):
    global vaild_angle, centric_distance, TUNNEL_VAILD, TUNNING_ENABLE, waypoint_yaw
    vaild_angle = tunnel.pose.pose.position.x
    centric_distance = tunnel.pose.pose.position.y
    TUNNEL_VAILD = tunnel.pose.pose.position.z
    if TUNNEL_VAILD:
        TUNNING_ENABLE = 1

rospy.init_node('drone_info')
rc_in = rospy.Subscriber('mavros/rc/in', RCIn, callback_rc, queue_size=1)
position_tag = rospy.Subscriber('apriltags/detections', AprilTagDetections, callback_tag, queue_size=1)
subodom = rospy.Subscriber('mavros/local_position/odom', Odometry, callback_odom, queue_size=1)
subtunnel = rospy.Subscriber('estimator/estimator', Odometry, callback_tunnel, queue_size=1)

pub_droneinfo =  rospy.Publisher('drone_info', DroneInfo, queue_size=1)

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    dinfo = DroneInfo()

    #Takeoff
    if SWITCH == 0:
        if (pos_fuse_z < tko_z):
            if TAKEOFF_ENABLE: 
                setpoint_x = tko_x
                setpoint_y = tko_y
                setpoint_z = tko_z
                setpoint_yaw = tko_yaw
                waypoint_yaw = tko_yaw
                TAKEOFF_ENABLE = False

    # if BACK_ADJ == 1 and pos_fuse_z > 0.1:
    if BACK_ADJ == 1:
        if TUNNING_ENABLE:
            #not adjust the yaw if drift is too small
            if abs(vaild_angle) > 0.1:
                waypoint_yaw = odom_yaw + vaild_angle
            
            #adjust when last has finished and drift if larger than 5 cm.
            if  abs(centric_distance) > 0.05:
                centric_set = centric_distance

            TUNNING_ENABLE = 0
        else: 
            pass
            # centric_set = 0
        if not TUNNING_FINISHED:
            setpoint_yaw = waypoint_yaw

    else:
        # SET_ONCE = 1
        # centric_set = 0
        pass

    #knob tunning left
    if KNOB_R == 2:
        feedback_mode = 0
        AUTO_ENABLE = True
    #knob tunning middle
    elif KNOB_R == 1 :
        feedback_mode = 0
        AUTO_ENABLE = False
    #knob tunning right, emergency slow moving mode
    elif KNOB_R == 0 :
        feedback_mode = 0
        AUTO_ENABLE = False
        setpoint_x = pos_fuse_x
        setpoint_y = pos_fuse_y
        OUT = True

    if KNOB_L == 2:
        if setpoint_x < MAX_FLY_RANGE and AUTO_ENABLE:
            if l2_distance(setpoint_x,setpoint_y,pos_fuse_x,pos_fuse_y) < 0.1: 
                waypoint_x = waypoint_x + 1
                waypoint_y = 0
    elif KNOB_L == 1:
        pass
    elif KNOB_L == 0:
        if setpoint_x < MAX_FLY_RANGE and AUTO_ENABLE :
            if l2_distance(setpoint_x,setpoint_y,pos_fuse_x,pos_fuse_y) < 0.1: 
                waypoint_x = waypoint_x - 1
                waypoint_y = 0

    #Trans to init yaw direction
    if AUTO_ENABLE:
        waypoint = np.array([waypoint_x, waypoint_y]).reshape(2, 1)
        centric = np.array([0, -centric_set]).reshape(2, 1)

        R_z_yaw = np.array([[np.cos(waypoint_yaw), -np.sin(waypoint_yaw)],
                            [np.sin(waypoint_yaw), np.cos(waypoint_yaw)]])

        waypoint = np.dot(R_z_yaw, waypoint)
        centric = np.dot(R_z_yaw, centric)

        setpoint_x = tko_x + waypoint[0][0] + centric[0][0]
        setpoint_y = tko_y + waypoint[1][0] + centric[1][0]
        # setpoint_x = tko_x  + centric_x
        # setpoint_y = tko_y  + centric_y

    if abs(waypoint_yaw - odom_yaw) < YAW_ADJ_TOR:
        TUNNING_FINISHED = 1
    else:
        TUNNING_FINISHED = 0

    # LANDING
    if SWITCH == 1:
        setpoint_z = 0
                   
    dinfo.header.stamp = rospy.get_rostime()
    dinfo.setpoint.position.x = setpoint_x
    dinfo.setpoint.position.y = setpoint_y
    dinfo.setpoint.position.z = setpoint_z
    dinfo.setpoint.orientation.w = setpoint_yaw
    dinfo.enable_setpoint = enable_setpoint
    dinfo.fmode = feedback_mode #VINS or TAG
    pub_droneinfo.publish(dinfo)


    print 'set_x',setpoint_x,'set_y',setpoint_y,'set_z',setpoint_z,'way_x',waypoint_x,'way_y', waypoint_y, 'tko_yaw', tko_yaw, 'odom_x', pos_fuse_x,'odom_y',pos_fuse_y
    print 'vaild_angle:', vaild_angle,'centric_distance ', centric_distance,' centric_set ',centric_set,'waypoint_yaw ',waypoint_yaw,'odom_yaw',odom_yaw
    print 'Tunnel_V',TUNNEL_VAILD,'T_enable ',TUNNING_ENABLE,'T_over ', TUNNING_FINISHED,'back ',BACK_ADJ,'SET_ONCE ',SET_ONCE


    rate.sleep()
