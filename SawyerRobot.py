#!/usr/bin/env python

# Sawyer Robot Class by Michail Theofanidis


import os
import numpy as np
import sympy as sp
import math
import cloudpickle
import pickle

from sympy.utilities.autowrap import autowrap

# Decleration of the Sawyer Robot Class
class Sawyer:
    def __init__(self):

        # If config file exits
        self.conf=0;

        # Number of joints, links and offsets
        self.num_joints = 7
        self.num_links= 8
        self.num_offsets= 7

        # Arrays that store the link parameters and offset parameters
        self.L = np.array([0.079, 0.237, 0.142, 0.259, 0.1264, 0.274, 0.105, 0.0695])
        self.d = np.array([0.081, 0.049, 0.14, 0.0419, 0.12249, 0.031, 0.109])

        # Joint Names array
        self.joint_names = ['right_j%i' % i for i in range(self.num_joints)]

        # Joint variable
        self.q = [sp.Symbol('q%i' % i) for i in range(self.num_joints)]
        self.dq = [sp.Symbol('dq%i' % i) for i in range(self.num_joints)]

        # Homogeneous Transformation Matrices of the Real Model 
        self.T_R_01 = sp.Matrix([[sp.cos(self.q[0]), -sp.sin(self.q[0]), 0, 0],
         [sp.sin(self.q[0]), sp.cos(self.q[0]), 0, 0],
         [0, 0, 1, self.L[0]],
         [0, 0, 0, 1]])

        self.T_R_12 = sp.Matrix([[sp.cos(self.q[1]), -sp.sin(self.q[1]), 0, self.d[0]],
         [0, 0, 1, self.d[1]],
         [-sp.sin(self.q[1]), -sp.cos(self.q[1]), 0, self.L[1]],
         [0, 0, 0, 1]])

        self.T_R_23 = sp.Matrix([[sp.cos(self.q[2]), -sp.sin(self.q[2]), 0, 0],
         [0, 0, -1, -self.d[2]],
         [sp.sin(self.q[2]), sp.cos(self.q[2]), 0, self.L[2]],
         [0, 0, 0, 1]])

        self.T_R_34 = sp.Matrix([[sp.cos(self.q[3]), -sp.sin(self.q[3]), 0, 0],
         [0, 0, 1, -self.d[3]],
         [-sp.sin(self.q[3]), -sp.cos(self.q[3]), 0, self.L[3]],
         [0, 0, 0, 1]])

        self.T_R_45 = sp.Matrix([[sp.cos(self.q[4]), -sp.sin(self.q[4]), 0, 0],
         [0, 0, -1, -self.d[4]],
         [sp.sin(self.q[4]), sp.cos(self.q[4]), 0, -self.L[4]],
         [0, 0, 0, 1]])

        self.T_R_56 = sp.Matrix([[sp.cos(self.q[5]), -sp.sin(self.q[5]), 0, 0],
         [0, 0, 1, self.d[5]],
         [-sp.sin(self.q[5]), -sp.cos(self.q[5]), 0, self.L[5]],
         [0, 0, 0, 1]])

        self.T_R_67 = sp.Matrix([[sp.cos(self.q[6]), -sp.sin(self.q[6]), 0, 0],
         [0, 0, -1, -self.d[6]],
         [sp.sin(self.q[6]), sp.cos(self.q[6]), 0, self.L[6]],
         [0, 0, 0, 1]])

        self.T_R_89 = sp.Matrix([[0,-1, 0, 0],
         [1, 0, 0, 0],
         [0, 0, 1, 0.0245],
         [0, 0, 0, 1]])

        self.T_R_9e = sp.Matrix([[1, 0, 0, 0],
         [0, 1, 0, 0],
         [0, 0, 1, 0.0450],
         [0, 0, 0, 1]])

        # Transformation Tree of the Real Model
        self.T_R=[self.T_R_01, self.T_R_12, self.T_R_23, self.T_R_34, self.T_R_45, self.T_R_56, self.T_R_67, self.T_R_89, self.T_R_9e]

        # DH Table of the Robot
        self.DH_Table = sp.Matrix([[self.d[0], -np.pi/2.0, 0, self.q[0]],
          [0, np.pi/2.0, self.d[1]+self.L[2], self.q[1]],
          [0, -np.pi/2.0, self.d[2]+self.L[3], self.q[2]],
          [0, np.pi/2.0, -(self.d[3]+self.L[4]), self.q[3]],
          [0, -np.pi/2.0, self.d[4]+self.L[5], self.q[4]],
          [0, np.pi/2.0, self.d[5]+self.L[6], self.q[5]],
          [0, 0, self.d[6]+self.L[7], self.q[6]]])

        # Homogeneous Transformation Matrices of the DH Model
        self.T_01=self.DH_Matrix(self.DH_Table[0,0],self.DH_Table[0,1],self.DH_Table[0,2],self.DH_Table[0,3])
        self.T_12=self.DH_Matrix(self.DH_Table[1,0],self.DH_Table[1,1],self.DH_Table[1,2],self.DH_Table[1,3])
        self.T_23=self.DH_Matrix(self.DH_Table[2,0],self.DH_Table[2,1],self.DH_Table[2,2],self.DH_Table[2,3])
        self.T_34=self.DH_Matrix(self.DH_Table[3,0],self.DH_Table[3,1],self.DH_Table[3,2],self.DH_Table[3,3])
        self.T_45=self.DH_Matrix(self.DH_Table[4,0],self.DH_Table[4,1],self.DH_Table[4,2],self.DH_Table[4,3])
        self.T_56=self.DH_Matrix(self.DH_Table[5,0],self.DH_Table[5,1],self.DH_Table[5,2],self.DH_Table[5,3])
        self.T_6e=self.DH_Matrix(self.DH_Table[6,0],self.DH_Table[6,1],self.DH_Table[6,2],self.DH_Table[6,3])

        # Transformation Tree of the DH Model
        self.T=[self.T_01, self.T_12, self.T_23, self.T_34, self.T_45, self.T_56, self.T_6e]

    # Method to adjust the offset in joint space
    def JointOffset(self,angles):

        angles[1]=angles[1]+math.radians(90)
        angles[6]=angles[6]+math.radians(170)+math.radians(90);

        return angles

    # Forward Kinematics for the Real Model
    def get_Tf_R(self):

        self.Tf_R=self.Forward_Kinematics(self.T_R,0)

        return self.Tf_R

    # Forward Kinematics for the DH Model and adding the missing length to the z coordinate    
    def get_Tf(self):

        self.Tf=self.Forward_Kinematics(self.T,1)

        return self.Tf

    # Homogeneous Transformation of the End Effector of the Real Model
    def get_Te_R(self):

        self.Te_R=cloudpickle.load(open('pickled_Te_R', 'rb'))

        return autowrap(self.Te_R,backend="cython",args=self.q)

    # Homogeneous Transformation of the End Effector of the DH Model   
    def get_Te(self):

        self.Te=cloudpickle.load(open('pickled_Te', 'rb'))

        return autowrap(self.Te,backend="cython",args=self.q)

    # Function to create the Homogeneous Transformation Matrices according to the DH Table 
    def DH_Matrix(self, a, alpha, delta, theta):

        self.dh_mat=sp.Matrix([[sp.cos(theta), -sp.sin(theta)*math.cos(alpha), sp.sin(theta)*math.sin(alpha), a*sp.cos(theta)],
                             [sp.sin(theta), sp.cos(theta)*math.cos(alpha), -sp.cos(theta)*math.sin(alpha), a*sp.sin(theta)],
                             [0, math.sin(alpha), math.cos(alpha), delta],
                             [0,0,0,1]])
        return self.dh_mat

    # Function that performs the Forward Kinematic Equations 
    def Forward_Kinematics(self, trans, diff):

        # If the matrices have
        if diff is 1 and self.conf is 1:
            self.temp=cloudpickle.load(open('pickled_Tf', 'rb'))

        if diff is 0 and self.conf is 1:
            self.temp=cloudpickle.load(open('pickled_Tf_R', 'rb'))

        if self.conf is 0:
            self.temp=[trans[0]]
            counter=-1;

            # Traverse through the transformation tree
            for i in trans[1:]:#

                counter=counter+1
                self.temp.append(self.temp[counter]*i)#

            if diff is 1:#

                counter=-1;
                for i in self.temp[:]:

                    counter=counter+1
                    self.temp[counter][2,3]=self.temp[counter][2,3]+self.L[0]+self.L[1]#

            #
            if diff is 1:
                cloudpickle.dump(self.temp, open('pickled_Tf', 'w'))
                cloudpickle.dump(self.temp[-1], open('pickled_Te', 'w'))
            else:
                cloudpickle.dump(self.temp, open('pickled_Tf_R', 'w'))
                cloudpickle.dump(self.temp[-1], open('pickled_Te_R', 'w'))

        if self.conf is 0:
            return self.temp
        return sp.lambdify(self.q, self.temp)









