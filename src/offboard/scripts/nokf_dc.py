#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8
#这个版本VINS的滤波早PIXHAWK里由EKF2完成

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
from offboard.msg import DroneInfo

from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import unscented_transform, MerweScaledSigmaPoints

set_z = 0
SWITCH = 2 #定义的额外开关
feedback_x = feedback_y = feedback_z = feedback_vz  =  0.0

#PID位置环输出的速度最大值，单位m/s，确保安全
MAX_VEL_X = 0.2
MAX_VEL_Y = 0.2
MAX_VEL_Z = 0.5

Z_FEEDBACK_AVALIABLE = 0
X_FEEDBACK_AVALIABLE = 0
Y_FEEDBACK_AVALIABLE = 0

current_state = State()


ENABLE_TAG_UKF = False #开启tagUKF滤波
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
TAG_X_BAIS = 2 #根据放置在中心时的偏置来确定
TAG_Y_BAIS = 0

setpoint_yaw = 0
setpoint_x = setpoint_y = setpoint_z = 0
enable_setpoint = 0
feedback_mode = 0 #deault: 0=VINS
KNOB_L = 0

vins_x = vins_y = vins_z = 0
MAX_RANGE = 3 #无人机最大移动范围，vins的估计不应该超过这个值
MAX_VEL_RANGE = 0.5 #无人机最大移动速度，vins的估计不应该超过这个值, tag的也不应该
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

    # 4.3KG  parameter
    P_x = 0.6 
    I_x = 0.0
    D_x = 0.0

    P_y = 0.6
    I_y = 0.0 
    D_y = 0.00

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
    global tag_lost_counter, tag_x, tag_y, tag_z, tag_seq, tag_time, KNOB_L,odom_pitch,odom_roll,odom_yaw, last_tag_x, last_tag_y, last_tag_time
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
            #通过旋钮KNOB_L来选择对准的tag
            if i.id == KNOB_L:
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
    # print 'after_x',local_tag_x, 'after_y',local_tag_y, 'after_z',local_tag_z
    # tag_vel_x = (local_tag_x - last_tag_x)/dt
    # tag_vel_y = (local_tag_y - last_tag_y)/dt
    last_tag_time = tag_time
    # print 'before_x',local_tag_x, 'before_y',local_tag_y, 'before_z',local_tag_z
    # print 'lost_counter and first tag:',tag_lost_counter,GET_FIRST_TAG,'odom_yaw',odom_yaw,'odom_pitch',odom_pitch,'odom_roll',odom_roll


def callback_vision(vis):
    global vins_x, vins_y, VINS_FEEDBACK_AVALIABLE
    vins_x = vis.pose.position.x
    vins_y = vis.pose.position.y
    if pos_fuse_x == 0 or pos_fuse_y == MAX_RANGE:
        VINS_FEEDBACK_AVALIABLE = 0
    else:
        VINS_FEEDBACK_AVALIABLE = 1   

def callback_odom(odom):
    global odom_roll,odom_pitch,odom_yaw, InitOdom, init_odom_roll, init_odom_pitch, init_odom_yaw,last_tag_seq, tag_seq,odom_last_time
    global ground_depth , pos_fuse_x, pos_fuse_y, pos_fuse_z, tag_time, incremental_pos_x, incremental_pos_y, incremental_pos_z
    global current_state, Z_FEEDBACK_AVALIABLE, Y_FEEDBACK_AVALIABLE, X_FEEDBACK_AVALIABLE
    global TAG_FEEDBACK_AVALIABLE, fix_tag_y,fix_tag_x, depth_init, TAKEOFF_FINISH, local_linear_x, local_linear_y, local_linear_z
    global  VINS_FEEDBACK_AVALIABLE
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


        #XY
        pos_fuse_x = odom.pose.pose.position.x
        pos_fuse_y = odom.pose.pose.position.y


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

        # print current_state.mode, current_state.armed,REACH_TOP, local_linear_xyz[2], Z_FEEDBACK_AVALIABLE, Z_FEEDBACK_AVALIABLE

        last_tag_seq = tag_seq
        odom_last_time = odom_time

        # print 'l_x:', body_linear_xyz[0],'l_y',body_linear_xyz[1], ';_z',body_linear_xyz[2]
        # print 'g_x:', local_linear_xyz[0],'g_y',local_linear_xyz[1], 'g_z',local_linear_xyz[2]
        # print 'roll',odom_roll,'pitch',odom_pitch,'yaw',odom_yaw


def callback_rc(rc):
    global SWITCH, KNOB_L
    if rc.channels[8] == 1065:
        SWITCH = 1
    if rc.channels[8] == 1933:
        SWITCH = 0
    if rc.channels[5] > 1065 and rc.channels[9] < 1355:
        KNOB_L = 0
    if rc.channels[5] >= 1355 and rc.channels[9] < 1644:
        KNOB_L = 1
    if rc.channels[5] >= 1644 and rc.channels[9] < 1933:
        KNOB_L = 2

def callback_state(st):
    global current_state
    current_state = st

def callback_info(info):
    global setpoint_x, setpoint_y, setpoint_z, setpoint_yaw, feedback_mode, enable_setpoint
    setpoint_x = info.setpoint.position.x
    setpoint_y = info.setpoint.position.y
    setpoint_z = info.setpoint.position.z
    setpoint_yaw = info.setpoint.orientation.w
    feedback_mode = info.fmode
    enable_setpoint = info.enable_setpoint

rospy.init_node('drone_control')
PIDinit()
UKFinit()
position_tag = rospy.Subscriber('apriltags/detections', AprilTagDetections, callback_tag, queue_size=1)
rc_in = rospy.Subscriber('mavros/rc/in', RCIn, callback_rc, queue_size=1)
sub_state = rospy.Subscriber('mavros/state', State, callback_state, queue_size=1)
subodom = rospy.Subscriber('mavros/local_position/odom', Odometry, callback_odom, queue_size=1)
subvision = rospy.Subscriber('mavros/vision_pose/pose', PoseStamped, callback_vision)
subinfo = rospy.Subscriber('drone_info', DroneInfo, callback_info, queue_size=1)
position_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

position_tag = rospy.Publisher('fuse_tag', PoseStamped, queue_size=1)


rate = rospy.Rate(100)  
while not rospy.is_shutdown():

    if TAG_FEEDBACK_AVALIABLE and ENABLE_TAG_UKF:
        ukf_input = [fix_tag_x, local_linear_x, fix_tag_y, local_linear_y]
        ukf.predict()
        ukf.update(ukf_input)
        tag_fuse_x = ukf.x[0]
        tag_fuse_y = ukf.x[2]
    elif TAG_FEEDBACK_AVALIABLE and not ENABLE_TAG_UKF:
        tag_fuse_x = local_tag_x
        tag_fuse_y = local_tag_y
    # print  "tag_y:", local_tag_y, "fix_tag_y:", fix_tag_y, "tag_fuse_y:", tag_fuse_y, "local_linear_y",local_linear_y
    vel_control = TwistStamped()
    pos_control = PoseStamped()

    pid_x.SetPoint = setpoint_x
    pid_y.SetPoint = setpoint_y
    pid_z.SetPoint = setpoint_z
    output_yaw = setpoint_yaw

    # feedback_mode = 1
    # X 位置PID
    if feedback_mode == 0:
        feedback_x = pos_fuse_x
    else:
        feedback_x = tag_fuse_x
    pid_x.update(feedback_x)
    output_x = pid_x.output

    # Y 位置PID
    if feedback_mode == 0:
        feedback_y = pos_fuse_y
    else:
        feedback_y = tag_fuse_y
    pid_y.update(feedback_y)
    output_y = pid_y.output

    # Z 位置PID
    feedback_z = pos_fuse_z
    pid_z.update(feedback_z)
    output_z = pid_z.output 



    #用速度限制防止无人机突然起降导致的不稳定
    if output_x > MAX_VEL_X:
        output_x = MAX_VEL_X
    elif output_x < -MAX_VEL_X:
        output_x = - MAX_VEL_X

    if output_y > MAX_VEL_Y:
        output_y = MAX_VEL_Y
    elif output_y < -MAX_VEL_Y:
        output_y = - MAX_VEL_Y
        
    if output_z > MAX_VEL_Z:
        output_z = MAX_VEL_Z
    elif output_z < -MAX_VEL_Z:
        output_z = - MAX_VEL_Z

    #反馈失效的时候保持静止
    if not Z_FEEDBACK_AVALIABLE:
        output_z = 0.00
    if not Y_FEEDBACK_AVALIABLE:
        output_y = 0.00
    if not X_FEEDBACK_AVALIABLE:
        output_x = 0.00

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
                                       PositionTarget.IGNORE_YAW_RATE  + PositionTarget.FORCE)
    raw_msg.header.stamp = stamp
    # output_x = 0.00
    # output_y = 0.00
    # output_z = 0.08
    # output_yaw = 0 
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
    if enable_setpoint:
        position_pub.publish(raw_msg)

    tag_fuse = PoseStamped()
    tag_fuse.header.stamp = stamp
    tag_fuse.pose.position.x = tag_fuse_x
    tag_fuse.pose.position.y = tag_fuse_y
    position_tag.publish(tag_fuse)

    #outter loop
    print  "x_set:", pid_x.SetPoint, "x_out:", output_x, "px_Feb:", feedback_x, "vx_Feb",local_linear_x 
    print  "y_set:", pid_y.SetPoint, "y_out:", output_y, "py_Feb:", feedback_y, "vy_Feb" ,local_linear_y
    print  "z_set:", pid_z.SetPoint, "z_out:", output_z, "pz_Feb:", feedback_z, "vz_Feb" ,local_linear_z ,'yaw_set',output_yaw ,'yaw_feb', odom_yaw,'pitch_feb', odom_pitch,'roll_feb', odom_roll

    rate.sleep()
