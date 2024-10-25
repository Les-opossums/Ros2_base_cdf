import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
import obj_detector.node_cluster as source

def distance(i, j, msg : LaserScan):
        return np.sqrt(float(msg.ranges[i])**2 + float(msg.ranges[j])**2 - 2*float(msg.ranges[i])*float(msg.ranges[j])*np.cos(np.abs((j-i))*float(msg.angle_increment)))

def calc_m1(segment,msg:LaserScan):
    return distance(segment[0],segment[1],msg)

def calc_alpha(segment,msg:LaserScan):
    delta_theta = msg.angle_increment
    nb_point = segment[1]-segment[0]

    theta = delta_theta*nb_point
    r0 = segment[0]
    r1 = segment[1]

    m1 = calc_m1(segment,msg)
    
    alpha = np.arccos((r0-2*r1*np.cos(theta))/m1)
    return alpha

def calc_theta_bis(segment,theta,msg:LaserScan):
    #return np.arccos(cos_alpha(segment,msg)) - theta - np.pi/2
    #return calc_alpha(segment,msg) - np.pi/2 
    return np.arccos(cos_alpha(segment,msg)) - np.pi/2

def cacl_theta_ex(segment,theta_bis,msg:LaserScan):
     return msg.angle_increment*segment[1]+theta_bis+msg.angle_min

def calc_rref(i,theta_bis,msg:LaserScan):
     return float(msg.ranges[i])*np.cos(theta_bis)

def cos_alpha(segment,msg:LaserScan):
    delta_theta = msg.angle_increment
    nb_point = segment[1]-segment[0]
    theta = delta_theta*nb_point
    r0 = float(msg.ranges[segment[0]])
    r1 = float(msg.ranges[segment[1]])
    m1 = calc_m1(segment,msg)
    cosalpha = ((m1**2)+(r0**2)-(r1**2))/(2*m1*r0)
    return cosalpha

def calc_pente(segment,msg:LaserScan):
    x_B = float(msg.ranges[segment[0]])*np.cos(msg.angle_increment*segment[0]+msg.angle_min)
    y_B = float(msg.ranges[segment[0]])*np.sin(msg.angle_increment*segment[0]+msg.angle_min)
    theta_bis = calc_theta_bis(segment,msg.angle_increment*np.abs(segment[0]-segment[1]),msg)
    m_ref = calc_rref(segment[0],theta_bis,msg)
    x_H = m_ref*np.cos(theta_bis+msg.angle_min)
    y_H = m_ref*np.sin(theta_bis+msg.angle_min)
    if np.abs(x_H-x_B) > 0.01 :
        A = (y_H-y_B)/(x_H-x_B)
        B = y_B - A*x_B
    else :
        A = 'cst'
        B = x_B
    return A,B