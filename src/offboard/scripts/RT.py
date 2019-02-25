#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8
import numpy as np

#T03代表从3转移到0坐标系下，T0C代表从相机坐标系转移到0坐标系下

def A_matrix(a,v):
    return np.array(
    [[np.cos(v), -np.sin(v), 0, a*np.cos(v)],
     [np.sin(v), np.cos(v),  0, a*np.sin(v)],
     [0,         0,          1, 0],
     [0,         0,          0, 1]])

def T01_matrix(a1,v1):
    return np.array(
    [[np.cos(v1), -np.sin(v1), 0, a1*np.cos(v1)],
     [np.sin(v1), np.cos(v1),  0, a1*np.sin(v1)],
     [0,         0,          1, 0],
     [0,         0,          0, 1]])

def T02_matrix(a1,a2, v1,v2):
    return np.array(
    [[np.cos(v1+v2), -np.sin(v1+v2), 0, a1*np.cos(v1) + a2*np.cos(v1+v2)],
     [np.sin(v1+v2), np.cos(v1+v2),  0, a1*np.sin(v1) + a2*np.sin(v1+v2)],
     [0,         0,          1, 0],
     [0,         0,          0, 1]])

def T03_matrix(a1,a2,a3, v1,v2,v3):
    return np.array(
    [[np.cos(v1+v2+v3), -np.sin(v1+v2+v3), 0, a1*np.cos(v1) + a2*np.cos(v1+v2) + a3*np.cos(v1+v2+v3)],
     [np.sin(v1+v2+v3), np.cos(v1+v2+v3),  0, a1*np.sin(v1) + a2*np.sin(v1+v2) + a3*np.sin(v1+v2+v3)],
     [0,         0,          1, 0],
     [0,         0,          0, 1]])

#v3以link2，y+为原点，和相机Z+朝向的夹角,左边正负，右边负
#v4是以机械臂旋转角度，左边正，右边负，水平朝上为0点，也可以理解成和相机坐标系Y+的夹角
# def T2C_matrix(v3,v4, L4x,L4y):
#     return np.array([
#         [np.cos(v3)*np.sin(v4), np.cos(v3)* np.cos(v4), -np.sin(v3),  L4x],
#         [np.sin(v3)*np.sin(v4), np.cos(v4)*np.sin(v3),  np.cos(v3),L4y],
#         [np.cos(v4),              -np.sin(v4),              0,           0],
#         [0,                          0,                           0,           1]
#     ])

# def T0C_matrix(a1, a2, v1,v2,v3,v4,L4x,L4y):
#     return np.dot(T02_matrix(a1,a2, v1,v2),T2C_matrix(v3,v4,L4x,L4y))

#TODO fix the trans matrix
# def RENU0_matrix(pitch,roll,yaw):
#     alpha = yaw
#     beta = pitch
#     gama = roll
#     return np.array([
#         [np.cos(alpha)*np.cos(beta), np.cos(alpha)*np.sin(beta)*np.sin(gama) - np.cos(gama)*np.sin(alpha), np.cos(alpha)*np.cos(gama)*np.sin(beta) + np.sin(alpha)*np.sin(gama)],
#         [np.cos(beta) * np.sin(alpha), np.cos(alpha)*np.cos(gama) + np.sin(alpha)*np.sin(beta)*np.sin(gama),np.cos(gama)*np.sin(alpha)*np.sin(beta)-np.cos(alpha)*np.sin(gama)],
#         [-np.sin(beta), np.cos(beta)*np.sin(gama), np.cos(beta)*np.cos(gama)]
#     ])

# def TENU0_matrix(pitch,roll,yaw,x=0,y=0,z=0):
#     alpha = yaw
#     beta = pitch
#     gama = roll
#     return np.array([
#         [np.cos(alpha)*np.cos(beta), np.cos(alpha)*np.sin(beta)*np.sin(gama) - np.cos(gama)*np.sin(alpha), np.cos(alpha)*np.cos(gama)*np.sin(beta) + np.sin(alpha)*np.sin(gama),x],
#         [np.cos(beta) * np.sin(alpha), np.cos(alpha)*np.cos(gama) + np.sin(alpha)*np.sin(beta)*np.sin(gama),np.cos(gama)*np.sin(alpha)*np.sin(beta)-np.cos(alpha)*np.sin(gama),y],
#         [-np.sin(beta), np.cos(beta)*np.sin(gama), np.cos(beta)*np.cos(gama),z],
#         [0,0,0,1]
#     ])

def dof2_position(v1,v2,L1,L2):
    x = L1*np.cos(v1) + L2 * np.cos(v1 + v2) 
    y = L1*np.sin(v1) + L2 * np.sin(v1 + v2)
    return  x, y

def dof3_position(v1,v2,L1,L2, L3x = 0.307,L3y = 0.039):
    x,y = dof2_position(v1,v2,L1,L2)
    x = x + L3x
    y = y + L3y
    return x,y

def T_GraspToDof2(grasp_x,grasp_y, L3x = 0.307,L3y = 0.039):
    dof2_x = grasp_x - L3x
    dof2_y = grasp_y - L3y
    return dof2_x, dof2_y
