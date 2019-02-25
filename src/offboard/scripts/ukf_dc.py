#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

#这个版本有VINS结合UKF和时间补偿

import rospy 
import roslib
import PID
import time
import tf
import numpy as np
from apriltags.msg import AprilTagDetections
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget, AttitudeTarget, State, RCIn
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PointStamped, Vector3, Vector3Stamped, TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints

set_z= 0
SWITCH = 2 #定义的额外开关
feedback_x = feedback_y = feedback_z = feedback_vz = feedback_yaw =  0.0
last_feedback_yaw = 0

#PID位置环输出的速度最大值，单位m/s，确保安全
MAX_VEL_X = 0.3
MAX_VEL_Y = 0.3
MAX_VEL_Z = 0.5

Z_FEEDBACK_AVALIABLE = 0
X_FEEDBACK_AVALIABLE = 0
Y_FEEDBACK_AVALIABLE = 0

current_state = State()
jump_height = 0
TKO_SPEED = 0.8
TOP_THRESH = 0.7
BOTTOM_THRESH = 0.1
REACH_TOP = False
JUMP_INIT = True
TAKEOFF_FINISH = False
DRONE_MOVE_RANGE = 1 

ENABLE_TAG_UKF = 1 #开启tagUKF滤波
output_yaw = 0
InitOdom = True
odom_roll = odom_pitch = odom_yaw = 0
init_odom_roll = init_odom_pitch = init_odom_yaw = 0
local_linear_x = local_linear_y = local_linear_z = 0

incremental_pos_x = incremental_pos_y = incremental_pos_z = 0
tag_lost_counter = 1
tag_x = tag_y = tag_z = 0
local_tag_x = local_tag_y = local_tag_z = 0
TAG_FEEDBACK_AVALIABLE = False
tag_fuse_x = tag_fuse_y = 0
fix_tag_y = fix_tag_x = 0
last_tag_x = last_tag_y = last_tag_time = 0
tag_vel_y = tag_vel_x = 0

ground_depth = -1 #初始化成-1代表无效值
depth_init = 0
tag_seq = last_tag_seq = 0
odom_last_time = 0
tag_time  = 0
pos_fuse_x = pos_fuse_y = pos_fuse_z = 0
body_pos_fuse_x = body_pos_fuse_y = 0
TAG_X_BAIS = 2 #根据放置在中心时的偏置来确定，用于滤波
TAG_Y_BAIS = 0
GET_FIRST_TAG = 2 #2代表初始化

KNOB = 0
VINS_DELAY = 0.150 #19ms 延时

InitVins = True
vins_x = vins_y = vins_z = 0
# 启动时校准震动导致的偏移，一般VINS会按照相机朝前为X+，但是如果要校准，摇动之后输出结果的时刻可能相机不能对其前方，因此将坐标系转移到此刻的相机朝前方向
init_vins_roll = init_vins_pitch = init_vins_yaw = 0
vins_seq = last_vins_seq = 0
vins_roll = vins_pitch = vins_yaw = 0
vins_to_ned_roll = vins_to_ned_yaw = vins_to_ned_pitch = 0
last_vins_x = last_vins_y = last_vins_z = 0
last_vins_time = 0
MAX_RANGE = 3 #无人机最大移动范围，vins的估计不应该超过这个值
MAX_VEL_RANGE = 0.5 #无人机最大移动速度，vins的估计不应该超过这个值, tag的也不应该
dt_list = []
vx_list = []
vy_list = []
fix_vins_x = fix_vins_y = 0
vins_vel_x = vins_vel_y = 0
VINS_FEEDBACK_AVALIABLE = 0

def f_cv(x, dt):
    """ state transition function for a
    constant velocity aircraft"""

    F = np.array([[1, dt, 0,  0],
                  [0,  1, 0,  0],
                  [0,  0, 1, dt],
                  [0,  0, 0,  1]], dtype=float)
    return np.dot(F, x)


def h_cv(x):
    return np.array([x[0], x[1], x[2], x[3]])

def UKFinit():
    global ukf
    ukf_fuse = []
    p_std_x = rospy.get_param('~p_std_x',0.005)
    v_std_x = rospy.get_param('~v_std_x',0.05)
    p_std_y = rospy.get_param('~p_std_y',0.005)
    v_std_y = rospy.get_param('~v_std_y',0.05)
    dt = rospy.get_param('~dt',0.01) #100HZ

    sigmas = MerweScaledSigmaPoints(4, alpha=.1, beta=2., kappa=-1.0)
    ukf = UKF(dim_x=4, dim_z=4, fx=f_cv, hx=h_cv, dt=dt, points=sigmas)
    ukf.x = np.array([0., 0., 0., 0.,])
    ukf.R = np.diag([p_std_x, v_std_x, p_std_y, v_std_y])
    ukf.Q[0:2, 0:2] = Q_discrete_white_noise(2, dt=dt, var=4.0)
    ukf.Q[2:4, 2:4] = Q_discrete_white_noise(2, dt=dt, var=4.0)
    ukf.P = np.diag([3, 1, 3, 1])

def PIDinit():
    global  pid_x, pid_y, pid_z

    # 3KG stable parameter
    # P_x = 0.2 
    # I_x = 0.0 
    # D_x = 0.0

    # P_y = 0.2
    # I_y = 0.0 
    # D_y = 0.0

    # P_z = 0.552
    # I_z = 0.02
    # D_z = 0.00

    # 4.3KG  parameter
    P_x = 0.2 
    I_x = 0.0 
    D_x = 0.0

    P_y = 0.2
    I_y = 0.0 
    D_y = 0.0

    P_z = 0.53
    I_z = 0.00
    D_z = 0.0005

    P_yaw = 0.0
    I_yaw = 0.0
    D_yaw = 0.0

    pid_x = PID.PID(P_x, I_x, D_x)
    pid_y = PID.PID(P_y, I_y, D_y)
    pid_z = PID.PID(P_z, I_z, D_z)

    #控制频率根据不同的node的速率来设定，ukf和odom的反馈都能达到200HZ，但是PID控制周期是依据OFFBOARD刷新频率确定的也就是100HZ
    pid_x.setSampleTime(0.01)
    pid_y.setSampleTime(0.01)
    pid_z.setSampleTime(0.01)


#apriltag的检测结果是按照相机为坐标系原点来检测的，结算的是目标板在相机的坐标系下的位置
def callback_tag(tag):
    global tag_lost_counter, tag_x, tag_y, tag_z, tag_seq, tag_time, KNOB,odom_pitch,odom_roll,odom_yaw, last_tag_x, last_tag_y, last_tag_time
    global tag_vel_y, tag_vel_x, local_tag_x, local_tag_y
    tag_time = tag.header.stamp.secs + tag.header.stamp.nsecs * 10**-9
    detected_tags = np.shape(tag.detections)[0]
    tag_seq = tag.header.seq
    if detected_tags == 1:        
        tag_lost_counter = 0
        tag_x = tag.detections[0].pose.position.x
        tag_y = tag.detections[0].pose.position.y
        tag_z = tag.detections[0].pose.position.z
    elif np.shape(tag.detections)[0] > 1:   
        for i in tag.detections:
            #通过旋钮KNOB来选择对准的tag
            if i.id == KNOB:
                tag_lost_counter = 0
                tag_x = i.pose.position.x
                tag_y = i.pose.position.y
                tag_z = i.pose.position.z 
                continue
    #id丢失，计数器加1
    else:
        tag_lost_counter  = tag_lost_counter + 1
        if tag_lost_counter > 1000:
            tag_lost_counter = 1000
    #将tag坐标系移动到ENU下
    # print 'before','x',tag_x, 'y',tag_y, 'z',tag_z
    R_x = np.array([[1, 0, 0], [0, np.cos(odom_roll), -np.sin(odom_roll)],[0, np.sin(odom_roll), np.cos(odom_roll)]])
    R_y = np.array([[np.cos(odom_pitch), 0, np.sin(odom_pitch )], [0, 1, 0], [-np.sin(odom_pitch), 0, np.cos(odom_pitch)]])
    R_z = np.array([[np.cos(odom_yaw), -np.sin(odom_yaw), 0], [np.sin(odom_yaw), np.cos(odom_yaw), 0], [0, 0, 1]])  
    #因为apriltag下面的坐标系的xyz和ENU并不对应，因此根据ENU 对应X+Y+Z+把tag的坐标系填进去，就是ENU的body frame了，再转换到ENU的local frame下
    body_tag_xyz = np.array([-tag_z, tag_x, tag_y]).reshape(3, 1)
    enu_tag_xyz = np.dot(np.dot(np.dot(R_z, R_y), R_x), body_tag_xyz)
    local_tag_x = enu_tag_xyz[0][0]
    local_tag_y = enu_tag_xyz[1][0]
    local_tag_z = enu_tag_xyz[2][0]  
    #用于时间上和IMU速度估计对齐
    dt = tag_time - last_tag_time       
    # print 'before_x',local_tag_x, 'before_y',local_tag_y, 'before_z',local_tag_z
    # tag_vel_x = (local_tag_x - last_tag_x)/dt
    # tag_vel_y = (local_tag_y - last_tag_y)/dt
    last_tag_time = tag_time
    # print 'before_x',local_tag_x, 'before_y',local_tag_y, 'before_z',local_tag_z
    # print 'lost_counter and first tag:',tag_lost_counter,GET_FIRST_TAG,'odom_yaw',odom_yaw,'odom_pitch',odom_pitch,'odom_roll',odom_roll

def callback_vins(vins):
    global vins_x, vins_y, vins_z, InitVins, vins_roll, vins_pitch, vins_yaw, init_vins_roll, init_vins_pitch, init_vins_yaw, vins_time, vins_seq
    global vins_to_ned_yaw, vins_to_ned_roll, vins_to_ned_pitch, last_vins_x, last_vins_y, last_vins_z, last_vins_time
    global local_linear_x, local_linear_y,vins_vel_x,vins_vel_y
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
        vins_seq = vins_seq + 1
        vins_time = vins.header.stamp.secs + vins.header.stamp.nsecs * 10**-9
        dt = vins_time - last_vins_time
        qn_vins = [vins.pose.pose.orientation.x, vins.pose.pose.orientation.y,
                   vins.pose.pose.orientation.z, vins.pose.pose.orientation.w]
        (vins_roll, vins_pitch, vins_yaw) = euler_from_quaternion(qn_vins)


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
        cam_body_vins_xyz = np.dot(np.dot(np.dot(R_z, R_y), R_x), body_vins_xyz)

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
        # print 'vins_x', vins_x, 'vins_y', vins_y, 'vins_z', vins_z

        vins_vel_x = (vins_x - last_vins_x)/dt
        vins_vel_y = (vins_y - last_vins_y)/dt
        vins_vel_z = (vins_z - last_vins_z)/dt

        last_vins_x = vins_x
        last_vins_y = vins_y
        last_vins_z = vins_z
        last_vins_time = vins_time

        # 超出移动范围直接判定数据无效，放着突然漂移导致的炸机
        if abs(vins_x) > MAX_RANGE or abs(vins_y) > MAX_RANGE:
            vins_x = 0
            vins_y = 0
            # print('WARNING! out of range may drift')

        # 超出速度限制，表明发生漂移
        if abs(vins_vel_x) > MAX_VEL_RANGE or abs(vins_vel_y) > MAX_VEL_RANGE:
            vins_x = 0
            vins_y = 0
            # print('WARNING! out of velocity range may drift')

        #估计速度和目前速度严重不符，表明估计错误
        # if abs(vins_vel_x)>local_linear_x * 2.5 and abs(local_linear_x) > 0.2:
        #     vins_x = 0
        # if abs(vins_vel_y)>local_linear_y * 2.5 and  abs(local_linear_y) > 0.2:
        #     vins_y = 0

def callback_odom(odom):
    global odom_roll,odom_pitch,odom_yaw, InitOdom, init_odom_roll, init_odom_pitch, init_odom_yaw,last_tag_seq, tag_seq,odom_last_time
    global ground_depth , pos_fuse_x, pos_fuse_y, pos_fuse_z, tag_time, incremental_pos_x, incremental_pos_y, incremental_pos_z
    global JUMP_INIT, jump_height, current_state, REACH_TOP, Z_FEEDBACK_AVALIABLE, Y_FEEDBACK_AVALIABLE, X_FEEDBACK_AVALIABLE
    global TAG_FEEDBACK_AVALIABLE, fix_tag_y,fix_tag_x, tag_pos_fuse_y, depth_init, body_pos_fuse_x, body_pos_fuse_y,TAKEOFF_FINISH, local_linear_x, local_linear_y, local_linear_z
    global vins_x, vins_y, vins_seq, VINS_DELAY, last_vins_seq, ukf, dt_list, vx_list, vy_list, fix_vins_x, fix_vins_y,vins_vel_x,vins_vel_y, VINS_FEEDBACK_AVALIABLE
    global tag_vel_x , tag_vel_y
    if InitOdom:
        odom_last_time = odom.header.stamp.secs + odom.header.stamp.nsecs * 10**-9
        qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        (init_odom_roll, init_odom_pitch, init_odom_yaw) = euler_from_quaternion(qn_odom)
        depth_init = odom.pose.pose.position.z
        InitOdom = False
    else:
        odom_time = odom.header.stamp.secs + odom.header.stamp.nsecs * 10**-9
        qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        (odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom)
        
        ground_depth = odom.pose.pose.position.z - depth_init
        #print 'ground_depth',ground_depth,'depth_init',depth_init

        # 减去初值
        # 可能在不水平的地面上起飞，所以roll和pitch其实不应该减去初始值
        # odom_roll = odom_roll - init_odom_roll
        # odom_pitch = odom_pitch - init_odom_pitch
        odom_yaw = odom_yaw - init_odom_yaw

        # 区间限制
        if odom_roll < -np.pi:
            odom_roll = odom_roll + 2 * np.pi
        if odom_roll > np.pi:
            odom_roll = odom_roll - 2 * np.pi

        if odom_pitch < -np.pi:
            odom_pitch = odom_pitch + 2 * np.pi
        if odom_pitch > np.pi:
            odom_pitch = odom_pitch - 2 * np.pi

        if odom_yaw < -np.pi:
            odom_yaw = odom_yaw + 2 * np.pi
        if odom_yaw > np.pi:
            odom_yaw = odom_yaw - 2 * np.pi

        # 求旋转矩阵
        R_x = np.array([ [1, 0, 0], [0, np.cos(odom_roll), -np.sin(odom_roll)], [0, np.sin(odom_roll), np.cos(odom_roll)] ])   
        R_y = np.array([ [np.cos(odom_pitch), 0, np.sin(odom_pitch)], [0, 1, 0], [-np.sin(odom_pitch), 0, np.cos(odom_pitch)]])   
        R_z = np.array([ [np.cos(odom_yaw), -np.sin(odom_yaw), 0], [np.sin(odom_yaw), np.cos(odom_yaw), 0], [0, 0, 1]])   

        # pixhawk速度坐标定义：箭头朝前为X+，机身水平超上为Z+，机身水平向左为Y+。速度是BODY frame,如果不做坐标变幻，那就一直在和pixhawk一致的坐标轴上
        # 速度分解
        #
        ###########################
        # 取反前速度坐标系
        # ^  x+ （飞空控前箭头）
        # 。
        # 。
        # 。
        # 。
        # 。
        # Z+(水平向上))。。。。。。>  y-
        ############################
        body_linear_xyz = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z]).reshape(3,1)
        # body_linear_xyz = np.transpose(body_linear_xyz)

        local_linear_xyz = np.dot(np.dot(np.dot(R_z, R_y), R_x),body_linear_xyz)

        local_linear_x = local_linear_xyz[0][0]
        local_linear_y = local_linear_xyz[1][0]
        local_linear_z = local_linear_xyz[2][0]
        
        #相机的xy随yaw改变，所以这里就不对Z进行旋转了
        #local_linear_xyz = np.dot(np.dot(R_y, R_x), body_linear_xyz)


        dt = odom_time - odom_last_time
        # TAG反馈处理
        #TAG反馈插值补偿
        if tag_seq - last_tag_seq == 0:
            incremental_pos_x = incremental_pos_x + local_linear_x * dt
            fix_tag_x = local_tag_x + incremental_pos_x

            incremental_pos_y = incremental_pos_y + local_linear_y * dt
            fix_tag_y = local_tag_y + incremental_pos_y 
        else:
            dt = odom_time - tag_time
            incremental_pos_x = 0
            incremental_pos_x = incremental_pos_x + local_linear_x * dt 
            fix_tag_x = local_tag_x + incremental_pos_x

            incremental_pos_y = 0
            incremental_pos_y = incremental_pos_y + local_linear_y * dt
            fix_tag_y = local_tag_y + incremental_pos_y


        if tag_lost_counter == 0:
            TAG_FEEDBACK_AVALIABLE = 1
        else:
            TAG_FEEDBACK_AVALIABLE = 0


        # 处理XY融合结果，XY
        #保留一个list的结果用于补偿
        dt_list.append(dt)
        vx_list.append(local_linear_x)
        vy_list.append(local_linear_y)
        if np.sum(np.array(dt_list)) >= VINS_DELAY:
            dt_list.pop(0)
            vx_list.pop(0)
            vy_list.pop(0)

        #vins位置没发生更新，当前位置等于上次滤波结果+fix值
        if vins_seq - last_vins_seq == 0:
            fix_vins_x = fix_vins_x + local_linear_x * dt
            fix_vins_y = fix_vins_y + local_linear_y * dt
        #vins发生跟新，当前位置等于更新的vins加上时间补偿 
        else:
            fix_vins_x = vins_x + np.sum(np.array(vx_list) * np.array(dt_list))
            fix_vins_y = vins_y + np.sum(np.array(vy_list) * np.array(dt_list))
            # fix_vins_x = vins_x 
            # fix_vins_y = vins_y
        #fix的位置和速度一起滤波得到pose_fuse

        if  vins_x == 0 or vins_y == 0:
            VINS_FEEDBACK_AVALIABLE = 0
        else:
            VINS_FEEDBACK_AVALIABLE = 1
      
        # 位置反馈有效性判断VINS可能会炸飞，所以要小心
        if  VINS_FEEDBACK_AVALIABLE or TAG_FEEDBACK_AVALIABLE:
            X_FEEDBACK_AVALIABLE = 1
            Y_FEEDBACK_AVALIABLE = 1
        else:
            X_FEEDBACK_AVALIABLE = 0
            Y_FEEDBACK_AVALIABLE = 0

        # Z反馈处理
        #优先选用单点激光进行高度估计
        if ground_depth > -0.5 and ground_depth < 6:
            Z_FEEDBACK_AVALIABLE = 1
            pos_fuse_z = ground_depth
        #所有反馈全部丢失，直接用imu估计
        else:
            Z_FEEDBACK_AVALIABLE = 0
            incremental_pos_z = incremental_pos_z + local_linear_xyz[2] * dt
            pos_fuse_z = pos_fuse_z + incremental_pos_z

        # print JUMP_INIT, current_state.mode, current_state.armed,REACH_TOP, local_linear_xyz[2], Z_FEEDBACK_AVALIABLE, jump_height,pos_fuse_z, Z_FEEDBACK_AVALIABLE
        if JUMP_INIT and current_state.mode == 'OFFBOARD' and local_linear_xyz[2] > TKO_SPEED * TOP_THRESH:
            REACH_TOP = True
     
        if REACH_TOP and JUMP_INIT and current_state.mode == 'OFFBOARD' and local_linear_xyz[2] < TKO_SPEED * BOTTOM_THRESH:
            if Z_FEEDBACK_AVALIABLE != 0:
                jump_height = pos_fuse_z
                TAKEOFF_FINISH = True
                JUMP_INIT = False



        last_tag_seq = tag_seq
        last_vins_seq = vins_seq 
        odom_last_time = odom_time

        # print 'l_x:', body_linear_xyz[0],'l_y',body_linear_xyz[1], ';_z',body_linear_xyz[2]
        # print 'g_x:', local_linear_xyz[0],'g_y',local_linear_xyz[1], 'g_z',local_linear_xyz[2]
        # print 'roll',odom_roll,'pitch',odom_pitch,'yaw',odom_yaw



def callback_rc(rc):
    global SWITCH,KNOB
    if rc.channels[8] == 1065:
        SWITCH = 1
    if rc.channels[8] == 1933:
        SWITCH = 0
    if rc.channels[5] > 1065 and rc.channels[9] < 1355:
        KNOB = 0
    if rc.channels[5] >= 1355 and rc.channels[9] < 1644:
        KNOB = 1
    if rc.channels[5] >= 1644 and rc.channels[9] < 1933:
        KNOB = 2

def callback_state(st):
    global current_state
    current_state = st
rospy.init_node('drone_control')
PIDinit()
UKFinit()
position_tag = rospy.Subscriber('apriltags/detections', AprilTagDetections, callback_tag, queue_size=1)
rc_in = rospy.Subscriber('mavros/rc/in', RCIn, callback_rc, queue_size=1)
sub_state = rospy.Subscriber('mavros/state', State, callback_state, queue_size=1)
subodom = rospy.Subscriber('mavros/local_position/odom', Odometry, callback_odom)
subvins = rospy.Subscriber('vins_estimator/odometry', Odometry, callback_vins)
# subdepth = rospy.Subscriber('depth', Odometry, callback_depth)
#pub_position = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)
#pub_velocity = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
position_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)


rate = rospy.Rate(100)  
while not rospy.is_shutdown():

    if TAG_FEEDBACK_AVALIABLE and ENABLE_TAG_UKF:
        ukf_input = [fix_tag_x, local_linear_x, fix_tag_y, local_linear_y]
        ukf.predict()
        ukf.update(ukf_input)
        tag_fuse_x = ukf.x[0]
        tag_fuse_y = ukf.x[2]

    
    print  "vins_y:", local_tag_y, "fix_vins_y:", fix_tag_y, "pos_fuse_y:", tag_fuse_y, "local_linear_y",local_linear_y


    vel_control = TwistStamped()
    pos_control = PoseStamped()

    #开关选择飞行位置
    if SWITCH == 1:
        # pid_z.SetPoint = jump_height
        pid_x.SetPoint = 0.0
        pid_y.SetPoint = 0.00
        pid_z.SetPoint = 0.25
    else:
        pid_x.SetPoint = 0.0
        pid_y.SetPoint = 0.0
        pid_z.SetPoint = 0.25

    # X 位置PID
    feedback_x = tag_fuse_x
    pid_x.update(feedback_x)
    output_x = pid_x.output

    # Y 位置PID
    feedback_y = tag_fuse_y
    pid_y.update(feedback_y)
    output_y = pid_y.output

    # Z 位置PID
    feedback_z = pos_fuse_z
    pid_z.update(feedback_z)
    output_z = pid_z.output 

   


    #用速度限制防止无人机突然起降导致的不稳定
    if abs(output_x) > MAX_VEL_X:
        output_x = MAX_VEL_X
    if abs(output_y) > MAX_VEL_Y:
        output_y = MAX_VEL_Y
    if abs(output_z) > MAX_VEL_Z:
        output_z = MAX_VEL_Z

    #反馈失效的时候保持静止
    if not Z_FEEDBACK_AVALIABLE:
        output_z = -0.01
    if not Y_FEEDBACK_AVALIABLE:
        output_y = 0.01
    if not X_FEEDBACK_AVALIABLE:
        output_x = 0.01

    #jump完成再开始控制
    # if TAKEOFF_FINISH != True:
    #     output_x = 0
    #     output_y = 0
    #     output_z = 0


    stamp = rospy.get_rostime()
    #暂时屏蔽yaw控制和y控制
    raw_msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED,
                             type_mask=
                                       PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY +  PositionTarget.IGNORE_PZ +
                                       PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ +
                                       PositionTarget.IGNORE_YAW_RATE  + PositionTarget.FORCE,
                             #加不加初始角还不确定，加初始角容易出现固定偏差，减去pi/2是因为会发生偏转
                             yaw = 0)
    raw_msg.header.stamp = stamp

    # output_x = 0.00
    # output_y = 0.00
    # output_z = 0.08
    output_yaw = -np.pi/2
    # if SWITCH == 1:
    #     output_yaw = -np.pi/2
    # else:
    #     output_yaw = 0
    #近地点不控制
    if feedback_z < 0.05:
        output_x = 0.0
        output_y = 0.0
    # 自由落体测试，整定速度PID专用MPC_Z_VEL_*
    # if feedback_z > 0.2:
    #     set_z = 1
    # if set_z:
    #     output_z = -0.08
    #     output_y = 0
    #     output_x = 0
    raw_msg.velocity.x = output_x
    raw_msg.velocity.y = output_y                             
    raw_msg.velocity.z = output_z
    raw_msg.yaw = output_yaw
    position_pub.publish(raw_msg)



    #outter loop
    # print  "x_set:", pid_x.SetPoint, "x_out:", output_x, "px_Feb:", feedback_x, "vx_Feb",local_linear_x, 'yaw_out',output_yaw  
    # print  "y_set:", pid_y.SetPoint, "y_out:", output_y, "py_Feb:", feedback_y, "vy_Feb" ,local_linear_y, 'yaw_out', output_yaw
    # print  "z_set:", pid_z.SetPoint, "z_out:", output_z, "pz_Feb:", feedback_z, "vz_Feb" ,local_linear_z ,'yaw_out',output_yaw 

    rate.sleep()


    # twist = TwistStamped()
    # twist.header.stamp = stamp
    # twist.twist.linear.x = 0
    # twist.twist.linear.y = 0
    # twist.twist.linear.z = 0.05
    # pub_velocity.publish(twist)