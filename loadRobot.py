from __future__ import print_function
import sys
from OCC.Voxel import Voxel_CollisionDetection
from OCC.IGESControl import IGESControl_Reader
from OCC.IFSelect import IFSelect_RetDone, IFSelect_ItemsByEntity
import time
from math import pi
from OCC.gp import *
from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.TopLoc import TopLoc_Location
from OCC.Display.SimpleGui import init_display
from OCC.TopoDS import TopoDS_Shape
import OCC.TopoDS
#************************************************

class loadRobot:
    def __init__(self,_folder,_names):
        self.folder=_folder
        self.file_names=_names
        self.file_list=[]
        self.Shapes=[]
        self.setFiles()
        self.pose_list=[]
        self.pose_list_generate()

    def setFiles(self):
        for i in range(7):
            self.file_list.append(self.folder+'/'+self.file_names[i])
            print('Robot file number ', i, ' is : ',self.file_list[i])

    def load(self):
        for i in range(7):
            print('Reading IGS file of link number ', i, ': ', self.file_list[i])
            Sh=self.readIGES(self.file_list[i])
            self.Shapes.append(Sh)
        print('Reading finished successfully')
        self.correct_poses()

    def readIGES(self, file):
        iges_reader = IGESControl_Reader()
        status = iges_reader.ReadFile(file)

        if status == IFSelect_RetDone:  # check status
            failsonly = False
            iges_reader.PrintCheckLoad(failsonly, IFSelect_ItemsByEntity)
            iges_reader.PrintCheckTransfer(failsonly, IFSelect_ItemsByEntity)
            ok = iges_reader.TransferRoots()
            aResShape = iges_reader.OneShape()
            return aResShape
        else:
            print("Error: can't read file:",file)
            return 0

    def correct_poses(self):
        for i in range(7):
            aCubeToploc = TopLoc_Location(self.pose_list[i])
            self.Shapes[i]=self.Shapes[i].Located(aCubeToploc)



    def pose_list_generate(self):
        trsf1=gp_Trsf()
        trsf2=gp_Trsf()
        trsf3=gp_Trsf()
        for i in range(7):
            trsf=gp_Trsf()
            self.pose_list.append(trsf)

        self.pose_list[0].SetTransformation(gp_Quaternion(gp_Vec(1, 0, 0), 3.1415 / 2), gp_Vec(0, 0, 0))
        self.pose_list[1].SetTransformation(gp_Quaternion(gp_Vec(1, 0, 0), 3.1415 / 2), gp_Vec(0, 0, -290))

        trsf1.SetTranslation(gp_Vec(0, 270, 59))
        trsf2.SetRotation(gp_Quaternion(gp_Vec(0, 0, 1), -3.1415 / 2))
        trsf2.Multiply(trsf1)
        self.pose_list[2].Multiply(trsf2)

        trsf1.SetRotation(gp_Quaternion(gp_Vec(0, 0, 1), -3.1415 / 2))
        trsf2.SetTranslation(gp_Vec(-66, -72, 0))
        trsf3.SetRotation(gp_Quaternion(gp_Vec(1, 0, 0), 3.1415))
        trsf2.Multiply(trsf1)
        trsf3.Multiply(trsf2)
        self.pose_list[3].Multiply(trsf3)

        trsf1.SetRotation(gp_Quaternion(gp_Vec(0, 1, 0), -3.1415 / 2))
        trsf2.SetRotation(gp_Quaternion(gp_Vec(0, 0, 1), -3.1415 / 2))
        trsf2.Multiply(trsf1)
        self.pose_list[4].Multiply(trsf2)

        trsf1.SetTranslation(gp_Vec(-302, 0, -630))
        trsf2.SetRotation(gp_Quaternion(gp_Vec(1, 0, 0), 3.1415 / 2))
        trsf3.SetRotation(gp_Quaternion(gp_Vec(0, 0, 1), 3.1415 / 2))
        trsf2.Multiply(trsf1)
        trsf3.Multiply(trsf2)
        self.pose_list[5].Multiply(trsf3)

        self.pose_list[6].SetTransformation(gp_Quaternion(gp_Vec(0, 0, 1), 3.1314), gp_Vec(0, 0, -65))












