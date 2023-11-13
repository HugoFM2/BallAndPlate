import numpy as np
from numpy import sin,cos,deg2rad
from CinematicaInversa.MatrixFunctions import rotate3d, translate3d

class Plate:
    def __init__(self,height):
        self.pointsDistance = 10 # Distance of points in relation to center, in cm
        self.anglePoints = 120 # Angle between points
        self.height = height
        
        self.radius = 15 # radius of plate

        self.xAngle = 0
        self.yAngle = 0
        
        self.centerPos = np.matrix([[0,
                                     0,
                                     self.height]]).T # Center of the plate, theoretically unchanged during all movements

        self.magnet1PosZero = np.matrix([[0 + self.centerPos.A1[0],
                                      self.pointsDistance + self.centerPos.A1[1],
                                      self.centerPos.A1[2]]]).T 
                                       
        self.magnet2PosZero = rotate3d(self.magnet1PosZero,0,0,deg2rad(-self.anglePoints)) # Calculated as 11 cm from center and rotated 120 between them

        self.magnet3PosZero = rotate3d(self.magnet1PosZero,0,0,deg2rad(-2*self.anglePoints))   
        
   
    def setHeight(self,height):     
        self.height = height
        self.centerPos = np.matrix([[0,
                                     0,
                                     self.height]]).T # Center of the plate, theoretically unchanged during all movements

        self.magnet1PosZero = np.matrix([[0 + self.centerPos.A1[0],
                                      self.pointsDistance + self.centerPos.A1[1],
                                      self.centerPos.A1[2]]]).T 
                                       
        self.magnet2PosZero = rotate3d(self.magnet1PosZero,0,0,deg2rad(-self.anglePoints)) # Calculated as 11 cm from center and rotated 120 between them

        self.magnet3PosZero = rotate3d(self.magnet1PosZero,0,0,deg2rad(-2*self.anglePoints))   


 
    def rotatePlate(self,angleX,angleY): # in Degrees
        self.xAngle = angleX
        self.yAngle = angleY
        
        oldPos = self.centerPos
        newcenterPos = rotate3d(self.centerPos,deg2rad(angleX),deg2rad(angleY),0)
        self.deltaPos = oldPos-newcenterPos
        
        self.magnet1Pos = rotate3d(self.magnet1PosZero,deg2rad(angleX),deg2rad(angleY),0)
        self.magnet1Pos = translate3d(self.magnet1Pos,self.deltaPos)
        
        self.magnet2Pos = rotate3d(self.magnet2PosZero,deg2rad(angleX),deg2rad(angleY),0)
        self.magnet2Pos = translate3d(self.magnet2Pos,self.deltaPos)
        
        self.magnet3Pos = rotate3d(self.magnet3PosZero,deg2rad(angleX),deg2rad(angleY),0)
        self.magnet3Pos = translate3d(self.magnet3Pos,self.deltaPos)
        
        

#     def rotatePlate(self,angleX,angleY): # Optmized
#         # Baseado no material escrito no caderno cinematica inversa
#         self.xAngle = angleX
#         self.yAngle = angleY
        
#         angleX = deg2rad(angleX)
#         angleY = deg2rad(angleY)
        
#         #Obter o novo centro e calcular seu delta, para trasladar depois
#         oldPos = self.centerPos
#         newcenterPos = rotate3d(self.centerPos,deg2rad(angleX),deg2rad(angleY),0)
#         self.deltaPos = oldPos-newcenterPos
        
#         magPosZero = [self.magnet1PosZero,self.magnet2PosZero,self.magnet3PosZero]
#         magPos = [1,2,3]
        
#         for i in range(len(magPos)): # Fazer com as 3 juntas
#             x,y,z = magPosZero[i].A1
            
#             magPos[i] = np.matrix([[x*cos(angleY)+z*sin(angleY)],
#                                    [x*sin(angleX)*sin(angleY)+y*cos(angleX)-z*sin(angleX)*cos(angleY)],
#                                    [-x*cos(angleX)*sin(angleY)+y*sin(angleX)+z*cos(angleX)*cos(angleY)]])

#             magPos[i] = translate3d(magPos[i],self.deltaPos)
# #             print(magPos[i])
            
#         self.magnet1Pos, self.magnet2Pos, self.magnet3Pos = magPos
    
