#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

import rospy
import time
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

InitVins = True
Received_odom = False
vins_x = vins_y = vins_z = 0
# 启动时校准震动导致的偏移，一般VINS会按照相机朝前为X+，但是如果要校准，摇动之后输出结果的时刻可能相机不能对其前方，因此将坐标系转移到此刻的相机朝前方向
init_vins_roll = init_vins_pitch = init_vins_yaw = 0
vins_seq = last_vins_seq = 0
vins_roll = vins_pitch = vins_yaw = 0
odom_roll = odom_pitch = odom_yaw = 0
vins_to_ned_roll = vins_to_ned_yaw = vins_to_ned_pitch = 0
last_vins_x = last_vins_y = last_vins_z = 0
last_vins_time = 0
MAX_RANGE = 5
MAX_VEL_RANGE = 1


def callback_vins(vins):
    global vins_x, vins_y, vins_z, InitVins, vins_roll, vins_pitch, vins_yaw, init_vins_roll, init_vins_pitch, init_vins_yaw, vins_time, vins_seq
    global vins_to_ned_yaw, vins_to_ned_roll, vins_to_ned_pitch, MAX_RANGE, MAX_VEL_RANGE, last_vins_x, last_vins_y, last_vins_z, last_vins_time
    if InitVins:
        vins_seq = vins.header.seq
        vins_time = vins.header.stamp.secs + vins.header.stamp.nsecs * 10**-9
        qn_vins = [vins.pose.pose.orientation.x, vins.pose.pose.orientation.y,
                   vins.pose.pose.orientation.z, vins.pose.pose.orientation.w]
        (init_vins_roll, init_vins_pitch,
         init_vins_yaw) = euler_from_quaternion(qn_vins)

        #应该在ENU下面发送，MAVROS会帮我转换成NED
        vins_to_ned_yaw = odom_yaw 
        vins_to_ned_roll = 0
        vins_to_ned_pitch = 0
        InitVins = False
    else:
        vins_seq = vins.header.seq
        vins_time = vins.header.stamp.secs + vins.header.stamp.nsecs * 10**-9
        dt = vins_time - last_vins_time
        qn_vins = [vins.pose.pose.orientation.x, vins.pose.pose.orientation.y,
                   vins.pose.pose.orientation.z, vins.pose.pose.orientation.w]
        (vins_roll, vins_pitch, vins_yaw) = euler_from_quaternion(qn_vins)

        # 减去初值
        # vins_roll = vins_roll - init_vins_roll
        # vins_pitch = vins_pitch - init_vins_pitch
        # vins_yaw = vins_yaw - init_vins_yaw

        # 区间限制
        # if vins_roll < -np.pi:
        #     vins_roll = vins_roll + 2 * np.pi
        # if vins_roll > np.pi:
        #     vins_roll = vins_roll - 2 * np.pi

        # if vins_pitch < -np.pi:
        #     vins_pitch = vins_pitch + 2 * np.pi
        # if vins_pitch > np.pi:
        #     vins_pitch = vins_pitch - 2 * np.pi

        # if vins_yaw < -np.pi:
        #     vins_yaw = vins_yaw + 2 * np.pi
        # if vins_yaw > np.pi:
        #     vins_yaw = vins_yaw - 2 * np.pi

        # print 'vins r:', init_vins_roll, 'vins p', init_vins_pitch, 'vins y', init_vins_yaw, 'vins_to_ned_yaw', vins_to_ned_yaw, 'vins_to_ned_roll', vins_to_ned_roll
        # print 'vins r:', vins_roll, 'vins p', vins_pitch, 'vins y', vins_yaw

        # 步骤1.先启动ins_estimator,开始校准，此时初始角可能产生偏差
        # 步骤2，启动dc.py此时用当前读取的角度，将校准时的world坐标系变换到启动时的坐标系
        R_x = np.array([[1, 0, 0], [0, np.cos(init_vins_roll), -np.sin(init_vins_roll)],
                        [0, np.sin(init_vins_roll), np.cos(init_vins_roll)]])
        R_y = np.array([[np.cos(init_vins_pitch), 0, np.sin(init_vins_pitch)], [
                       0, 1, 0], [-np.sin(init_vins_pitch), 0, np.cos(init_vins_pitch)]])
        R_z = np.array([[np.cos(init_vins_yaw), -np.sin(init_vins_yaw), 0],
                        [np.sin(init_vins_yaw), np.cos(init_vins_yaw), 0], [0, 0, 1]])

        body_vins_xyz = np.array(
            [vins.pose.pose.position.x, vins.pose.pose.position.y, vins.pose.pose.position.z]).reshape(3, 1)
        # 这种旋转相当于以初始坐标点为参考
        cam_body_vins_xyz = np.dot(
            np.dot(np.dot(R_z, R_y), R_x), body_vins_xyz)

        # R_x_NED = np.array([ [1, 0, 0], [0, np.cos(vins_to_ned_roll), -np.sin(vins_to_ned_roll)], [0, np.sin(vins_to_ned_roll), np.cos(vins_to_ned_roll)] ])
        R_y_NED = np.array([[np.cos(vins_to_ned_pitch), 0, np.sin(vins_to_ned_pitch)], [
                           0, 1, 0], [-np.sin(vins_to_ned_pitch), 0, np.cos(vins_to_ned_pitch)]])
        R_z_NED = np.array([[np.cos(vins_to_ned_yaw), -np.sin(vins_to_ned_yaw), 0],
                            [np.sin(vins_to_ned_yaw), np.cos(vins_to_ned_yaw), 0], [0, 0, 1]])

        # local_vins_xyz = np.dot(R_z_NED, cam_body_vins_xyz)

        local_vins_xyz = np.dot(np.dot(R_z_NED, R_y_NED), cam_body_vins_xyz)

        vins_x = local_vins_xyz[0][0]
        vins_y = local_vins_xyz[1][0]
        vins_z = local_vins_xyz[2][0]

        # print 'body_x', body_vins_xyz[0], 'body_y', body_vins_xyz[1], 'body_z',body_vins_xyz[2]


        vins_vel_x = (vins_x - last_vins_x)/dt
        vins_vel_y = (vins_y - last_vins_y)/dt
        vins_vel_z = (vins_z - last_vins_z)/dt
        last_vins_x = vins_x
        last_vins_y = vins_y
        last_vins_z = vins_z
        last_vins_time = vins_time

        print 'vins_x', vins_x, 'vins_y', vins_y, 'vins_z', vins_z, 'velx',vins_vel_x, 'vely',vins_vel_y, last_vins_x, last_vins_y

        # 超出移动范围直接判定数据无效，放着突然漂移导致的炸机
        if abs(vins_x) > MAX_RANGE or abs(vins_y) > MAX_RANGE:
            vins_x = 0
            vins_y = 0
            print('WARNING! out of range may drift')

        # 超出速度限制，表明发生漂移
        if abs(vins_vel_x) > MAX_VEL_RANGE or abs(vins_vel_y) > MAX_VEL_RANGE:
            vins_x = 0
            vins_y = 0
            print('WARNING! out of velocity range may drift')

        UAVPose = PoseStamped()
        UAVPose.header.stamp = rospy.Time.now()
        UAVPose.pose.position.x = vins_x
        UAVPose.pose.position.y = vins_y
        # UAVPose.pose.position.z = vins_z

        UAVPose.pose.orientation.x = vins.pose.pose.orientation.x
        UAVPose.pose.orientation.y = vins.pose.pose.orientation.y
        UAVPose.pose.orientation.z = vins.pose.pose.orientation.z
        UAVPose.pose.orientation.w = vins.pose.pose.orientation.w

        if vins_x == 0 or vins_y == 0:
            print('DROPPING THIS FRAME!')
        else:
            pub_pose.publish(UAVPose)




def callback_odom(odom):
    global Received_odom, odom_roll, odom_pitch, odom_yaw
    Received_odom = True
    qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
               odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    (odom_roll, odom_pitch, odom_yaw) = euler_from_quaternion(qn_odom)
    # print odom_yaw


rospy.init_node('pose_dog')

subvins = rospy.Subscriber('vins_estimator/odometry', Odometry, callback_vins)
pub_pose = rospy.Publisher('mavros/vision_pose/pose',PoseStamped, queue_size=1)
subodom = rospy.Subscriber(
    'mavros/local_position/odom', Odometry, callback_odom)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    # 如果没有开始接收ins，那么先用虚拟帧触发EKF
    if InitVins and not Received_odom:
        UAVPose = PoseStamped()
        UAVPose.header.stamp = rospy.Time.now()
        UAVPose.pose.position.x = 0.0
        UAVPose.pose.position.y = 0.0
        UAVPose.pose.position.z = 0.0
        pub_pose.publish(UAVPose)

    rate.sleep()
