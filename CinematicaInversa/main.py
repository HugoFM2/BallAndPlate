from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/
import numpy as np
from numpy import sin,cos,deg2rad

from Servo import Servo
from Plate import Plate
from BallAndPlate import BallAndPlate
import time



pca = ServoKit(channels=16)
Servo1 = Servo(pca,servoIndex=0,zeroAngle=14,servoPos=np.matrix([[0,10,0]]).T)
Servo2 = Servo(pca,servoIndex=1,zeroAngle=10,servoPos=np.matrix([[10*sin(deg2rad(120)),10*cos(deg2rad(120)),0]]).T)
Servo3 = Servo(pca,servoIndex=2,zeroAngle=12,servoPos=np.matrix([[10*sin(deg2rad(240)),10*cos(deg2rad(240)),0]]).T)

Plate = Plate()

sistema = BallAndPlate(Servo1,Servo2,Servo3,Plate)

# for i in range(11,15):
# 	print(i)
# sistema.Servo1.setAngle(90)
# time.sleep(1)
# sistema.Servo1.setAngle(0)
sistema.setHeight(12)
# sistema.printStatus()
print(sistema.Plate.magnet1Pos)
time.sleep(1)

# sistema.setAngle(15,0)
# # sistema.printStatus()
# print(sistema.Plate.magnet1Pos)

# time.sleep(2)
# sistema.setAngle(-15,0)
# sistema.printStatus()
# print(sistema.Plate.magnet1Pos)

# time.sleep(2)
# sistema.setPlateAngle(0,-15)
# time.sleep(1)
# sistema.setPlateAngle(15,0)
# time.sleep(1)
# sistema.setPlateAngle(-15,0)
	# time.sleep(1)
# sistema.Servo1.setAngle(90)

# for i in range(5):
# 	for j in range(120,300,1):
# 		sistema.setHeight(0.05*j)
# 		time.sleep(0.001)
# 	for j in range(300,120,-1):
# 		sistema.setHeight(0.05*j)	
# 		time.sleep(0.001)

for i in range(0,1000):
	angulo = round(15*sin(0.08*i),2)
	angulo2 = round(15*cos(0.08*i),2)
	print(f'angulo: {angulo}')
	sistema.setAngle(angulo2,angulo)
	time.sleep(0.002)

time.sleep(2)