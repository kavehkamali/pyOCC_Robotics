from __future__ import print_function
import sys
import numpy as np
from math3D import *
from pyquaternion import Quaternion
from copy import deepcopy, copy

nrOfLinks = 6

class DH_table:
    alpha=np.zeros(nrOfLinks)
    a=np.zeros(nrOfLinks)
    d=np.zeros(nrOfLinks)
    theta_0=np.zeros(nrOfLinks)

class frame:
    def __init__(self):
        self.H=np.eye(4)
        self.rH=np.eye(4)

class link:
    def __init__(self):
        self.frameA=frame()
        self.frameB=frame()

class robot_kinematics:
    def __init__(self):
        self.dh_table=DH_table()
        self.Links =[link() for i in range(nrOfLinks+1)]
        self.theta = np.zeros(nrOfLinks)
        self.theta_home = np.zeros(nrOfLinks)
        IRB120_dh = DH_table()
        IRB120_dh.alpha =   np.array( [0, -90,   0, -90, 90, -90] )
        IRB120_dh.a =       np.array( [0,   0, 270,  70,  0,   0] )
        IRB120_dh.d =       np.array( [290, 0,   0, 302,  0, 130] )
        IRB120_dh.theta_0 = np.array( [0, -90,   0,   0,  0, 180] )
        self.setDH(IRB120_dh)
        self.goHome()

    def setDH(self, _dh_table):
        _dh_table.alpha= _dh_table.alpha * np.pi / 180
        _dh_table.theta_0= _dh_table.theta_0 * np.pi / 180
        self.dh_table= _dh_table

    def setJoints(self, _theta):
        _theta= _theta * np.pi / 180
        self.theta=deepcopy(_theta)
        self.solveFK()

    def goHome(self):
        self.theta = deepcopy(self.theta_home)
        self.solveFK()

    def solveFK(self):
        for i in range(1,7):
            H_alpha = np.eye(4)
            H_alpha[0:3,0:3] = AngleAxis(self.dh_table.alpha[i - 1], np.matrix('1;0;0'))
            H_a = np.eye(4)
            H_a[0,3] = self.dh_table.a[i - 1]
            H_theta = np.eye(4)
            H_theta[0:3,0:3]= AngleAxis(self.dh_table.theta_0[i - 1] + self.theta[i - 1], np.matrix('0;0;1'))
            H_d = np.eye(4)
            H_d[2, 3] = self.dh_table.d[i - 1]
            self.Links[i].frameB.H = deepcopy(self.Links[i - 1].frameB.H.dot(H_alpha.dot(H_a).dot(H_theta).dot(H_d)))

    def get_link_H(self,i):
        return self.Links[i].frameB.H

    def get_link_R(self,i):
        H = self.Links[i].frameB.H
        R = H[0:3,0:3]
        return R

    def get_link_p(self, i):
        H = self.Links[i].frameB.H
        p = H[0:3,3]
        return p

    def get_link_q(self, i):
        H = self.Links[i].frameB.H
        Q=Quaternion(matrix=H)
        return Q.elements

    def FK(self, _theta):
        self.setJoints(_theta)
        return self.Links[nrOfLinks].frameB.H

    def get_xyz(self):
        H = self.Links[nrOfLinks].frameB.H
        return H[0:3,3]

    def get_q(self, i):
        H = self.Links[nrOfLinks].frameB.H
        Q=Quaternion(matrix=H)
        return Q.elements

