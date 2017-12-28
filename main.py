from __future__ import print_function
import numpy as np
import sys
from OCC.Voxel import Voxel_CollisionDetection
from OCC.IGESControl import IGESControl_Reader
from OCC.IFSelect import IFSelect_RetDone, IFSelect_ItemsByEntity
import time
from math import pi
from OCC.gp import gp_Ax1, gp_Pnt, gp_Dir, gp_Trsf
from OCC.TopLoc import TopLoc_Location
from OCC.Quantity import *
from OCC.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeVertex)
from OCC.Display.SimpleGui import init_display
from loadRobot import loadRobot
from robot_kinematics import robot_kinematics
from copy import *
from OCC.AIS import AIS_Shape


class form:
    def __init__(self):
        self.display, self.start_display, self.add_menu, self.add_function_to_menu = init_display()
        self.load = 0
        self.robotLinks=[]
        self.ais_robotlinks=[]
        self.current_trsf=[gp_Trsf() for i in range(7)]
        self.last_trsf = [gp_Trsf() for i in range(7)]
        self.robotKin=robot_kinematics()
        self.identity_trsf=gp_Trsf()
        self.H_last = [np.eye(4) for i in range(7)]

    def load_robot(self):
        robotModel=loadRobot('./models',['base.igs','link1.igs','link2.igs','link3.igs','link4.igs','link5.igs','link6.igs'])
        robotModel.load()
        self.robotLinks=deepcopy(robotModel.Shapes)
        for i in range(7):
            self.ais_robotlinks.append(self.display.DisplayShape(self.robotLinks[i], update=True))

        #self.move_robot_Shapes(np.array([0, 0, 0, 0, 0, 0]))

    def move(self):
        for j in range(100):
            self.move_robot_AIS(np.array([j*5, -j*0.1, 0, 0, 0, 0]))


    def move_robot_AIS(self,J):
        self.robotKin.setJoints(J)
        for i in range(1, 7):
            H = self.robotKin.get_link_H(i)
            self.current_trsf[i].SetValues( H[0, 0], H[0, 1], H[0, 2], H[0, 3], H[1, 0], H[1, 1], H[1, 2], H[1, 3], H[2, 0], H[2, 1], H[2, 2], H[2, 3])
            self.display.Context.SetLocation( self.ais_robotlinks[i], TopLoc_Location(self.current_trsf[i]) )
        self.display.Context.UpdateCurrentViewer()

    def move_robot_Shapes(self,J):
        self.display.Context.EraseAll()
        self.robotKin.setJoints(J)
        for i in range(0, 7):
            H = self.robotKin.get_link_H(i)
            self.last_trsf[i].SetValues(self.H_last[i][0, 0], self.H_last[i][0, 1], self.H_last[i][0, 2], self.H_last[i][0, 3],
                                        self.H_last[i][1, 0], self.H_last[i][1, 1], self.H_last[i][1, 2], self.H_last[i][1, 3],
                                        self.H_last[i][2, 0], self.H_last[i][2, 1], self.H_last[i][2, 2], self.H_last[i][2, 3])
            self.current_trsf[i].SetValues(H[0, 0], H[0, 1], H[0, 2], H[0, 3], H[1, 0], H[1, 1], H[1, 2], H[1, 3],
                                           H[2, 0], H[2, 1], H[2, 2], H[2, 3])
            self.robotLinks[i].Move( TopLoc_Location(self.current_trsf[i].Multiplied(self.last_trsf[i].Inverted()) ))
            self.ais_robotlinks[i] = self.display.DisplayShape(self.robotLinks[i], update=True)
            self.H_last[i] = deepcopy(H)

    def drawOrigin(self):
        XEdge = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(1000, 0, 0))
        YEdge = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(0, 1000, 0))
        ZEdge = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(0, 0, 1000))
        self.display.DisplayColoredShape(XEdge.Edge(), 'RED')
        self.display.DisplayColoredShape(YEdge.Edge(), 'GREEN')
        self.display.DisplayColoredShape(ZEdge.Edge(), 'BLUE')

    def command(self):
        i=1
        while i==1:
            command=input(">>>")
            if command=='move':
                J = np.array([float(x) for x in input(">>>").split()])
                print(J)
                self.move_robot_Shapes(J)
                i=0

if __name__ == '__main__':
    new_form = form()
    new_form.add_menu('Tools')
    new_form.add_function_to_menu('Tools', new_form.drawOrigin)
    new_form.add_function_to_menu('Tools', new_form.load_robot)
    new_form.add_function_to_menu('Tools', new_form.command)
    new_form.add_function_to_menu('Tools', new_form.move)
    new_form.start_display()











"""
    def move_robot_ShapesB(self,J):
        self.robotKin.setJoints(J)
        for i in range(0, 7):
            H = self.robotKin.get_link_H(i)
            self.last_trsf[i].SetValues(self.H_last[i][0, 0], self.H_last[i][0, 1], self.H_last[i][0, 2], self.H_last[i][0, 3],
                                        self.H_last[i][1, 0], self.H_last[i][1, 1], self.H_last[i][1, 2], self.H_last[i][1, 3],
                                        self.H_last[i][2, 0], self.H_last[i][2, 1], self.H_last[i][2, 2], self.H_last[i][2, 3])
            self.current_trsf[i].SetValues(H[0, 0], H[0, 1], H[0, 2], H[0, 3], H[1, 0], H[1, 1], H[1, 2], H[1, 3],
                                           H[2, 0], H[2, 1], H[2, 2], H[2, 3])
            self.robotLinks[i].Move( TopLoc_Location(self.current_trsf[i].Multiplied(self.last_trsf[i].Inverted()) ))
            self.display.Context.SetLocation(self.ais_robotlinks[i], TopLoc_Location(self.current_trsf[i]))
            self.H_last[i] = deepcopy(H)
        self.display.Context.UpdateCurrentViewer()


    def move_robot_ShapesC(self,J):
        #self.display.Context.EraseAll()
        self.robotKin.setJoints(J)
        for i in range(0, 7):
            H = self.robotKin.get_link_H(i)
            self.last_trsf[i].SetValues(self.H_last[i][0, 0], self.H_last[i][0, 1], self.H_last[i][0, 2], self.H_last[i][0, 3],
                                        self.H_last[i][1, 0], self.H_last[i][1, 1], self.H_last[i][1, 2], self.H_last[i][1, 3],
                                        self.H_last[i][2, 0], self.H_last[i][2, 1], self.H_last[i][2, 2], self.H_last[i][2, 3])
            self.current_trsf[i].SetValues(H[0, 0], H[0, 1], H[0, 2], H[0, 3], H[1, 0], H[1, 1], H[1, 2], H[1, 3],
                                           H[2, 0], H[2, 1], H[2, 2], H[2, 3])
            self.robotLinks[i].Move( TopLoc_Location(self.current_trsf[i].Multiplied(self.last_trsf[i].Inverted()) ))
            self.ais_robotlinks[i]=AIS_Shape(self.robotLinks[i])
            self.display.Context.Update(self.ais_robotlinks[i])
            ##self.ais_robotlinks[i] = self.display.DisplayShape(self.robotLinks[i], update=True)
            self.H_last[i] = deepcopy(H)
        self.display.Context.UpdateCurrentViewer()
"""