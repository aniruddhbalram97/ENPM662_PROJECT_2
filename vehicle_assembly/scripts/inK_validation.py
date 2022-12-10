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

# Calculation of Jacobian Matrix
def Jacobian_calc(H0_1,H0_2,H0_3,H0_4,H0_5,H0_6):
    #The xdot,ydot and zdot contributions are initially calculated and that 3X6 matrix stored in J_lin
    J_lin_0 =Matrix(np.cross(np.matrix([[0],[0],[1]]),np.matrix([[H0_6[3]],[H0_6[7]],[H0_6[11]]]),axis=0))
    J_lin_1 =Matrix(np.cross(np.matrix([[H0_1[2]],[H0_1[6]],[H0_1[10]]]),(np.matrix([[H0_6[3]],[H0_6[7]],[H0_6[11]]])-np.matrix([[H0_1[3]],[H0_1[7]],[H0_1[11]]])),axis = 0))
    J_lin_2 =Matrix(np.cross(np.matrix([[H0_2[2]],[H0_2[6]],[H0_2[10]]]),(np.matrix([[H0_6[3]],[H0_6[7]],[H0_6[11]]])-np.matrix([[H0_2[3]],[H0_2[7]],[H0_2[11]]])),axis = 0))
    J_lin_3 =Matrix(np.cross(np.matrix([[H0_4[2]],[H0_4[6]],[H0_4[10]]]),(np.matrix([[H0_6[3]],[H0_6[7]],[H0_6[11]]])-np.matrix([[H0_4[3]],[H0_4[7]],[H0_4[11]]])),axis = 0))
    J_lin_4 =Matrix(np.cross(np.matrix([[H0_5[2]],[H0_5[6]],[H0_5[10]]]),(np.matrix([[H0_6[3]],[H0_6[7]],[H0_6[11]]])-np.matrix([[H0_5[3]],[H0_5[7]],[H0_5[11]]])),axis = 0))
    J_lin_5 =Matrix(np.cross(np.matrix([[H0_6[2]],[H0_6[6]],[H0_6[10]]]),(np.matrix([[H0_6[3]],[H0_6[7]],[H0_6[11]]])-np.matrix([[H0_6[3]],[H0_6[7]],[H0_6[11]]])),axis = 0))
    J_lin = Matrix(np.concatenate((J_lin_0,J_lin_1,J_lin_2,J_lin_3,J_lin_4,J_lin_5),axis = 1))

    #The omega_x,omega_y,omega_z contributions are calculated separately and that 3X6 matrix is stored in J_rot
    J_rot_0 = Matrix(np.matrix([[0],[0],[1]]))
    J_rot_1 =Matrix(np.matrix([[H0_1[2]],[H0_1[6]],[H0_1[10]]]))
    J_rot_2 =Matrix(np.matrix([[H0_2[2]],[H0_2[6]],[H0_2[10]]]))
    J_rot_3 =Matrix(np.matrix([[H0_4[2]],[H0_4[6]],[H0_4[10]]]))
    J_rot_4 =Matrix(np.matrix([[H0_5[2]],[H0_5[6]],[H0_5[10]]]))
    J_rot_5 =Matrix(np.matrix([[H0_6[2]],[H0_6[6]],[H0_6[10]]]))

    J_rot = Matrix(np.concatenate((J_rot_0,J_rot_1,J_rot_2,J_rot_3,J_rot_4,J_rot_5),axis = 1))
    #J_lin and J_rot are concatenated to form the 6X6 Jacobian matrix
    Jacobian = Matrix(np.concatenate((J_lin, J_rot),axis=0))
    print("\nJacobian : \n")
    pprint(Jacobian)
    print("\n")
    return Jacobian

# Obtain the end-effector velocity
def getEnd_eff_vel(v,angle):
    #Function to calculate End Effector velocity vector with its linear and rotational components
    #Parameters : v-> linear velocity to cover the circumference.
               #  angle-> angle covered in the ith iteration

    

    #as x = 0.679, y = rcos(theta) and z = rsin(theta), vx =0, vy=-rsin(theta)*dx/dt=-rsin(theta)*w=-vsin(theta)
    #and vz=rsin(theta)dz/dt = rsin(theta)*w=vsin(theta).
    #vy=0 as motion is planar. similarly, all roational vel. is taken as zero.
    vel=np.array([0,-v*np.sin(angle),v*np.cos(angle), 0, 0, 0])
    return vel

# Update angles based on the formula
def update_theta_joint(theta_joint,q_dot,del_t):
    #Given current theta values and the joint velocity, calculate the joint velocity at the next time step
    #Parameters:
    #   theta_joint-> current joint angle values
    #    q_dot -> joint velocities
    #    del_t -> time elapsed since last time step

    return Array([theta_joint[0]+q_dot[0,0]*del_t,theta_joint[1]+q_dot[1,0]*del_t,theta_joint[2]+q_dot[2,0]*del_t,theta_joint[3]+q_dot[3,0]*del_t,\
                theta_joint[4]+q_dot[4,0]*del_t,theta_joint[5]+q_dot[5,0]*del_t])

# Define a talker node to publish angle values 
def talker(theta_joint):
    pub_ur3_joint_1 = rospy.Publisher('/ur3_joint_1_controller/command', Float64, queue_size=10)
    pub_ur3_joint_2 = rospy.Publisher('/ur3_joint_2_controller/command', Float64, queue_size=10)
    pub_ur3_joint_5 = rospy.Publisher('/ur3_joint_5_controller/command', Float64, queue_size=10)
    pub_ur3_joint_8 = rospy.Publisher('/ur3_joint_8_controller/command', Float64, queue_size=10)
    pub_ur3_joint_9 = rospy.Publisher('/ur3_joint_9_controller/command', Float64, queue_size=10)
    pub_ee_joint = rospy.Publisher('/ee_joint_controller/command', Float64, queue_size=10)
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(100)
    pub_ur3_joint_1.publish(theta_joint[0])
    pub_ur3_joint_2.publish(-theta_joint[1])
    pub_ur3_joint_5.publish(theta_joint[2])
    pub_ur3_joint_8.publish(theta_joint[3])
    pub_ur3_joint_9.publish(-theta_joint[4])
    pub_ee_joint.publish(-theta_joint[5])
    rospy.sleep(0.5)


if __name__ == '__main__':
    # All joint angles are assumed to be 0 degrees.
    d0 = 0.1519
    d3 = 0.11235
    d4 = 0.08535
    d5 = 0.0819
    a1 = -0.24365
    a2 = -0.21325
    #Initial joint angles 
    theta_joint = Array([0,0,0,0,0,0])
    
    # Angle resolution of moving joints, in degrees
    angle_step=9

    #The t will be used to iterate for each position of the end effector. As we are starting from the top of the circle,
    #the angle measured will start from 90 degrees. Hence i+90.
    t=[i+90 for i in range(0,360,angle_step)]
    len_arr=len(t) # Number of angle steps

    radius=0.5 #radius of circle  
    
    # Time taken to draw the circle, in s
    time_taken=10.0

    # Calculate linear velocity to cover circumference in the given time 
    v=np.pi*radius/(4*time_taken)

    # Time step between iterations
    del_t=time_taken/len_arr
    
    for i in t:
        #Calculating each transformation matrix H0_n iteratively.
        H0_1=DH_Calc_UR(math.pi/2,0,d0,theta_joint[0],1)
        H1_2=DH_Calc_UR(0,a1,0,theta_joint[1],2)
        H0_2= H0_1*H1_2
        H2_3=DH_Calc_UR(0,a2,0,theta_joint[2],3)
        H0_3= H0_1*H1_2*H2_3
        H3_4=DH_Calc_UR(math.pi/2,0,d3,theta_joint[3],4)
        H0_4= H0_1*H1_2*H2_3*H3_4
        H4_5=DH_Calc_UR(-math.pi/2,0,d4,theta_joint[4],5)
        H0_5=H0_1*H1_2*H2_3*H3_4*H4_5
        H5_6=DH_Calc_UR(0,0,d5,theta_joint[5],6)
        H0_6= H0_1*H1_2*H2_3*H3_4*H4_5*H5_6
        #Transformstion matrix for transformation from mobile base to manipulator base
        base =  Matrix([[1 ,0, 0, 0.163],
                        [0, math.cos(math.pi), -math.sin(math.pi),0],
                        [0,math.sin(math.pi),math.cos(math.pi),0.279],
                        [0,0,0,1]])
        Teff = base*H0_6
        print("\n Teff:")
        pprint(Teff)
        print("\n") 
        #Initial end effector position.The motion is planar in x-z frame. Hence Y coordinates are not considered.
        End_eff_x_pos = Teff[0,3]
        End_eff_z_pos = Teff[2,3]
        End_eff_y_pos = Teff[1,3]
        print("y:",End_eff_y_pos,"\n")
        print("z:",End_eff_z_pos,"\n")
        print("x:",End_eff_x_pos,"\n")
        #Caculating Jacobian
        Jacobian = Jacobian_calc(H0_1,H0_2,H0_3,H0_4,H0_5,H0_6)

        # Take the inverse of the Jacobian
        Jacobian=np.matrix(Jacobian,dtype='float64')
        Jinv=np.linalg.pinv(Jacobian)
        #print("Jacobian_inverse :\n")
        #pprint(Matrix(Jinv))

        #Get end effector velocity in polar coordinates based on linear velocity
        # i contains the next angular value (in degrees)
        End_eff_vel=getEnd_eff_vel(v,i*math.pi/180)

        #Perform inverse kinematics and get joint velocities
        q_dot=Jinv*(End_eff_vel.reshape((6,1)))

        # Calculate joint angles for the next iteration
        # This incremented value will be used when H0_7 is calculated in next iteration and then its (x,z) is plotted
        theta_joint=update_theta_joint(theta_joint,q_dot,del_t)
        talker(theta_joint)
        #print("theta_joint:\n")
        #pprint(theta_joint)

        # Plot the current end effector coordinates
        plt.scatter(End_eff_y_pos,End_eff_z_pos)
    plt.show()

    