#!/usr/bin/python3
import sympy
from sympy.matrices import Matrix
from sympy import sin,cos
from sympy import symbols,expand
from sympy import pprint, matrix_multiply_elementwise
from sympy import *
import math
import matplotlib.pyplot as plt
import numpy as np

ax = plt.figure().add_subplot(projection='3d')
#Defining the necessary symbols
#θ1,θ2,θ3,θ4,θ5,θ6,θ7 are the joint angles -> theta_joint DH_calc_Panda
#d1,d3,d5,d7 are the offsets between the current and previous joints along the common normal -> dz in DH_calc_Panda
#a is the length of the common normals along z between two consecutive joints -> a in DH_calc_Panda
θ1,θ2,θ3,θ4,θ5,θ6,θ7,d1,d3,d5 = symbols("θ1 θ2 θ3 θ4 θ5 θ6 θ7 d1,d3 d5")
d0 = 0.1519
d3 = 0.11235
d4 = 0.08535
d5 = 0.0819
# a for the 4th,5th and 7th joints are assumed to be equal. i.e a4,a5,a7 = 0.0880
a1 = -0.24365
a2 = -0.21325
#alpha is the angle between the prvious joint's z-axis and current joint's z-axis about the common normal(x-axis).
#i depicts the ith joint 
def DH_calc_Panda(alpha,a,dz,theta_joint,i):
    Trans = Matrix([[math.cos(theta_joint),-math.sin(theta_joint)*math.cos(alpha),math.sin(theta_joint)*math.sin(alpha),a*math.cos(theta_joint)],
                    [math.sin(theta_joint),math.cos(theta_joint)*math.cos(alpha),-math.cos(theta_joint)*math.sin(alpha),a*math.sin(theta_joint)],
                    [0,math.sin(alpha),math.cos(alpha),dz],
                    [0,0,0,1]])
    return Trans


Points_X = []
Points_Y = []
Points_Z = []
P_ = Matrix([0, 0, 0, 1])
i = 0
it = 3
Random_array = np.random.uniform(low=-math.pi, high=pi, size=(it,))
for theta_0 in range(it):
   # if(i == 10000):
     #   break
    H0_1=DH_calc_Panda(math.pi/2.0,0,d0,Random_array[theta_0],1)

    for theta_1 in range(it):
   # if(i == 10000):
     #   break
        H1_2=DH_calc_Panda(0,a1,0,Random_array[theta_1],2)
        H0_2= H0_1*H1_2

        for theta_2 in range(it):
   # if(i == 10000):
     #   break
            H2_3=DH_calc_Panda(0,a1,0,Random_array[theta_2],2)
            H0_3= H0_2*H2_3

            for theta_3 in range(it):
   # if(i == 10000):
     #   break
                H3_4=DH_calc_Panda(0,a1,0,Random_array[theta_3],2)
                H0_4 = H0_3 * H3_4
            
                for theta_4 in range(it):
   # if(i == 10000):
     #   break
                    H4_5=DH_calc_Panda(0,a1,0,Random_array[theta_4],2)
                    H0_5= H0_4 * H4_5

                    for theta_5 in range(it):
   # if(i == 10000):
     #   break
                        H5_6=DH_calc_Panda(0,a1,0,Random_array[theta_5],2)
                        H0_6= H0_5 * H5_6
                        P1 = H0_6 * P_
                        print(P1)
                        Points_X.append(P1[0])
                        Points_Y.append(P1[1])
                        Points_Z.append(P1[2])
                        i = i + 1

ax.scatter(Points_X, Points_Y, Points_Z, label='points in (x, z)')
plt.show()

if __name__ == '_main_':
    print(Points_X, "\n")
    print(Points_Y,"\n")
    print(Points_Z,"\n")