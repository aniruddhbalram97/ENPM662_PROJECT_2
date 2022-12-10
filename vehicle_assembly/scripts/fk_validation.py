#!/usr/bin/python3

# Importing the libraries
import rospy
import sympy
import math
from sympy.matrices import Matrix
from sympy import sin,cos
from sympy import symbols,expand
from sympy import pprint, matrix_multiply_elementwise
from sympy import *
import numpy as np
import matplotlib.pyplot as plt 
from std_msgs.msg import Float64

#Defining the necessary symbols
#θ1,θ2,θ3,θ4,θ5,θ6,θ7 are the joint angles -> theta_joint DH_Calc_UR
#d1,d3,d5,d7 are the offsets between the current and previous joints along the common normal -> dz in DH_Calc_UR
#a is the length of the common normals along z between two consecutive joints -> a in DH_Calc_UR
θ1,θ2,θ3,θ4,θ5,θ6,θ7,d1,d3,d5,d7 = symbols("θ1 θ2 θ3 θ4 θ5 θ6 θ7 d1 d3 d5 d7")

#alpha is the angle between the prvious joint's z-axis and current joint's z-axis about the common normal(x-axis).
#i depicts the ith joint 
def DH_Calc_UR(alpha,a,dz,theta_joint,i):
    Trans = Matrix([[math.cos(theta_joint),-math.sin(theta_joint)*math.cos(alpha),math.sin(theta_joint)*math.sin(alpha),a*math.cos(theta_joint)],
                    [math.sin(theta_joint),math.cos(theta_joint)*math.cos(alpha),-math.cos(theta_joint)*math.sin(alpha),a*math.sin(theta_joint)],
                    [0,math.sin(alpha),math.cos(alpha),dz],
                    [0,0,0,1]])
    return Trans

# Defining initial theta values
theta1 = Matrix([0,0,0,0,0,0])

# Defining final theta values
theta2 = Matrix([0, math.pi/2, math.pi/2, math.pi/3, 0, 0])

# DH Parameters of our robot
d0 = 0.1519
d3 = 0.11235
d4 = 0.08535
d5 = 0.0819
a1 = -0.24365
a2 = -0.21325

point_base = Matrix([0, 0, 0, 1])
# Obtaining transformations using FK transformation matrix for initial angles
H0_1=DH_Calc_UR(math.pi/2,0,d0,0,1)
H1_2=DH_Calc_UR(0,a1,0,0,2)
H0_2= H0_1*H1_2
H2_3=DH_Calc_UR(0,a2,0,0,3)
H0_3= H0_1*H1_2*H2_3
H3_4=DH_Calc_UR(math.pi/2,0,d3,0,4)
H0_4= H0_1*H1_2*H2_3*H3_4
H4_5=DH_Calc_UR(-math.pi/2,0,d4,0,5)
H0_5=H0_1*H1_2*H2_3*H3_4*H4_5
H5_6=DH_Calc_UR(0,0,d5,0,6)
H0_6= H0_1*H1_2*H2_3*H3_4*H4_5*H5_6
point_init = H0_6 * point_base

print('point_init')
pprint(point_init)


# Obtaining transformation using FK transformation matrix for final angles
H0_1=DH_Calc_UR(math.pi/2,0,d0,0,1)
H1_2=DH_Calc_UR(0,a1,0,math.pi/2,2)
H0_2= H0_1*H1_2
H2_3=DH_Calc_UR(0,a2,0,math.pi/2,3)
H0_3= H0_1*H1_2*H2_3
H3_4=DH_Calc_UR(math.pi/2,0,d3,math.pi/3,4)
H0_4= H0_1*H1_2*H2_3*H3_4
H4_5=DH_Calc_UR(-math.pi/2,0,d4,0,5)
H0_5=H0_1*H1_2*H2_3*H3_4*H4_5
H5_6=DH_Calc_UR(0,0,d5,0,6)
H0_6= H0_1*H1_2*H2_3*H3_4*H4_5*H5_6
point_final = H0_6 * point_base

print('point_fin')
pprint(point_final)
# creating a talker node to publish final velocities to the joints
def talker():
    pub_ur3_joint_1 = rospy.Publisher('/ur3_joint_1_controller/command', Float64, queue_size=10)
    pub_ur3_joint_2 = rospy.Publisher('/ur3_joint_2_controller/command', Float64, queue_size=10)
    pub_ur3_joint_5 = rospy.Publisher('/ur3_joint_5_controller/command', Float64, queue_size=10)
    pub_ur3_joint_8 = rospy.Publisher('/ur3_joint_8_controller/command', Float64, queue_size=10)
    pub_ur3_joint_9 = rospy.Publisher('/ur3_joint_9_controller/command', Float64, queue_size=10)
    pub_ee_joint = rospy.Publisher('/ee_joint_controller/command', Float64, queue_size=10)
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100)
    pub_ur3_joint_1.publish(0)
    pub_ur3_joint_2.publish(math.pi/2)
    pub_ur3_joint_5.publish(math.pi/2)
    pub_ur3_joint_8.publish(math.pi/3)
    pub_ur3_joint_9.publish(0)
    pub_ee_joint.publish(0)
    rospy.sleep(0.5)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        talker()