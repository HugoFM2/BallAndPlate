import CinematicaInversa.Servo,CinematicaInversa.Plate
from PyQt5.QtCore import *
from PyQt5.QtGui import * 

from CinematicaInversa.MatrixFunctions import rotate3d, translate3d
import numpy as np
from numpy import sin,cos,deg2rad,rad2deg


class BallAndPlate(QThread):
	def __init__(self,Servo1,Servo2,Servo3,Plate,remote=False,remoteType='UDP'):
		QThread.__init__(self)
		self.Servo1 = Servo1
		self.Servo2 = Servo2
		self.Servo3 = Servo3
		self.Plate = Plate

		self.remote = remote
		self.remoteType = remoteType

		if self.remote:
			if remoteType == 'UDP':
				from UDPCommunication.client import UDPClient
				self.client = UDPClient() # Importa comunicacao UDP caso seja remoto
			if remoteType == 'Serial':
				from UDPCommunication.serial import SerialComm 
				self.arduino = SerialComm('COM8')



	def setHeight(self,height):
		self.Plate.setHeight(height)
		self.setAngle(self.Plate.xAngle, self.Plate.yAngle)


	def setAngle(self,alfa,beta): # Alfa beta in degrees
		# Ajusta angulo da plataforma para obter angulos desejados dos imas
		self.Plate.rotatePlate(alfa,beta)


		# Obtem a melhor resposta para a junta J2 e ajusta o angulo do servo
		resServo1 = self.getPossiblePositionJ2(self.Plate.magnet1Pos,self.Servo1.pos,0,lastGamma=deg2rad(0))
		self.Servo1.setAngle(rad2deg(resServo1['BestGamma']))

		resServo2 = self.getPossiblePositionJ2(self.Plate.magnet2Pos,self.Servo2.pos,deg2rad(-120),lastGamma=deg2rad(0))
		self.Servo2.setAngle(rad2deg(resServo2['BestGamma']))
		# print("Angle Servo2:",rad2deg(resServo2['BestGamma']))

		resServo3 = self.getPossiblePositionJ2(self.Plate.magnet3Pos,self.Servo3.pos,deg2rad(-240),lastGamma=deg2rad(0))
		self.Servo3.setAngle(rad2deg(resServo3['BestGamma']))

		if self.remote:
			if self.remoteType == 'UDP':
				self.client.send(self.Servo1.actualAngle,self.Servo2.actualAngle,self.Servo3.actualAngle)
			elif self.remoteType == 'Serial':
				self.arduino.send(self.Servo1.actualAngle,self.Servo2.actualAngle,self.Servo3.actualAngle)


	def getPossiblePositionJ2(self,magPos,servoPos,zAngle,lastGamma):
		#Obtem as 2 respostas possiveis da junta 2 dado a posicao desejada e a posicao do servo
		# Para isso, o servo é "desrotacionado" para aplicar a equacao apenas em 2 dimensoes, ignorando o eixo x
		
		#Desrotacionando os pontos
		unrotatedMagPos = rotate3d(magPos,0,0,-zAngle)
		unrotatedServoPos = rotate3d(servoPos,0,0,-zAngle)
		
		#OBtem a equacao de 2o grau e as 2 respostas possiveis
		x1,y1,z1 = unrotatedMagPos.A1
		r1 = 8.8
		
		x2,y2,z2 = unrotatedServoPos.A1
		r2 = 7
		
		cy = (y1-y2)/(z1-z2)
		dy = ( (r1**2-r2**2) - (y1**2-y2**2) - (z1**2-z2**2) )/(2*(z1-z2))
		
		a = (1+cy**2)
		b = (-2*y1+2*cy*dy+2*z1*cy)
		c = (y1**2 + dy**2 + 2*z1*dy + z1**2 - r1**2)
		
		delta = (b**2) - (4*a*c)
		y_1 = (-b+np.sqrt(delta))/(2*a)
		z_1 = -cy*y_1-dy
		res1 = (y_1,z_1)
		
		y_2 = (-b-np.sqrt(delta))/(2*a)
		z_2 = -cy*y_2-dy
		res2 = (y_2,z_2)
		
		# Calcula qual é a melhor posicao(Determina qual é o menor deltaGamma)
		Gamma1 = np.arcsin((z_1-z2)/7)
		Gamma2 = np.arcsin((z_2-z2)/7)

		
		if (abs(lastGamma-Gamma1) < abs(lastGamma-Gamma2)): # 
		# if Gamma1 < Gamma2:
			bestPos = np.matrix([[0],
						  [res1[0]],
						  [res1[1]]])
			
			otherPos = np.matrix([[0],
						  [res2[0]],
						  [res2[1]]])  
			# print('---------') 
			# print(Gamma1)
			# Rotaciona novamente as posicoes para obter a posicao real
			bestPos = rotate3d(bestPos,angle_x=deg2rad(0),angle_y=deg2rad(0),angle_z=zAngle)
			otherPos = rotate3d(otherPos,angle_x=deg2rad(0),angle_y=deg2rad(0),angle_z=zAngle)    
			return {"BestPos"    : bestPos,
					"OtherPos"   : otherPos,
					"BestGamma"  : Gamma1,
					"OtherGamma" : Gamma2}
		
		else:
			bestPos = np.matrix([[0],
						  [res2[0]],
						  [res2[1]]])
			
			otherPos = np.matrix([[0],
						  [res1[0]],
						  [res1[1]]])   
			# print('---------') 
			# print(rad2deg(Gamma2))
			# Rotaciona novamente as posicoes para obter a posicao real
			bestPos = rotate3d(bestPos,angle_x=deg2rad(0),angle_y=deg2rad(0),angle_z=zAngle)
			otherPos = rotate3d(otherPos,angle_x=deg2rad(0),angle_y=deg2rad(0),angle_z=zAngle)    
			return {"BestPos"    : bestPos,
					"OtherPos"   : otherPos,
					"BestGamma"  : Gamma2,
					"OtherGamma" : Gamma1}   



	def printStatus(self):
		print('------------------')
		print(f'Servo1: {round(self.Servo1.angle,2)} / Servo2: {round(self.Servo2.angle,2)} / Servo3: {round(self.Servo3.angle,2)}')
		print(f'Alfa: {round(self.Plate.xAngle,2)} / Beta: {round(self.Plate.yAngle,2)}')
		print('------------------')