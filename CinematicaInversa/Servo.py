#Libraries
import time    #https://docs.python.org/fr/3/library/time.html
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/
import numpy as np
from numpy import sin,cos,deg2rad

class Servo:
	def __init__(self,pca,servoIndex,servoPos,minAngle=10,maxAngle=170,zeroAngle=0):
		self.minAngle=minAngle
		self.maxAngle=maxAngle

		self.pca = pca 
		self.servoIndex = servoIndex
		self.pos = servoPos
		self.zeroAngle = zeroAngle
		self.actualAngle = zeroAngle



		self.min_imp = 500
		self.max_imp = 2500

		self.pca.servo[servoIndex].set_pulse_width_range(self.min_imp , self.max_imp)
		

	def setAngle(self,angle):
		if angle + self.zeroAngle > self.maxAngle:
			angle = self.maxAngle
		if angle + self.zeroAngle< self.minAngle:
			angle = self.minAngle

		self.pca.servo[self.servoIndex].angle = angle + self.zeroAngle
		self.actualAngle = angle


		


# print("Inicializando servo, aguarde 3s")
# pca = ServoKit(channels=16)
# Servo1 = Servo(pca,servoIndex=0,zeroAngle=14,servoPos=np.matrix([[0,10,0]]).T)
# Servo2 = Servo(pca,servoIndex=1,zeroAngle=10,servoPos=np.matrix([[10*sin(deg2rad(120)),10*cos(deg2rad(120)),0]]).T)
# Servo3 = Servo(pca,servoIndex=2,zeroAngle=12,servoPos=np.matrix([[10*sin(deg2rad(240)),10*cos(deg2rad(240)),0]]).T)
# time.sleep(3)

# print("Mudando Angulos")
# for i in range(180,0,-1):
# 	Servo1.setAngle(i*0.5)
# 	Servo2.setAngle(i*0.5)
# 	Servo3.setAngle(i*0.5)
# 	time.sleep(0.005)