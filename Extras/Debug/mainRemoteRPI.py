from UDPCommunication.Server import UDPServer
import numpy as np

from CinematicaInversa.Servo import Servo
from CinematicaInversa.Plate import Plate
from CinematicaInversa.BallAndPlate import BallAndPlate

from numpy import sin,cos,deg2rad
import time
from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/


try:
	
	pca = ServoKit(channels=16)
	Servo1 = Servo(servoIndex=0,zeroAngle=14,servoPos=np.matrix([[0,10,0]]).T,pca=pca)
	Servo2 = Servo(servoIndex=1,zeroAngle=10,servoPos=np.matrix([[10*sin(deg2rad(120)),10*cos(deg2rad(120)),0]]).T,pca=pca)
	Servo3 = Servo(servoIndex=2,zeroAngle=12,servoPos=np.matrix([[10*sin(deg2rad(240)),10*cos(deg2rad(240)),0]]).T,pca=pca)


	server = UDPServer()
	server.start()
	while True:
		
		if server.data is not None:
			s1_angle,s2_angle,s3_angle = server.data

			Servo1.setAngle(s1_angle)
			Servo2.setAngle(s2_angle)
			Servo3.setAngle(s3_angle)

			# print(f'AngleX:{x} / AngleY: {y}')
		# else:
			# ballAndPlate.setAngle(0,0)
		time.sleep(0.001)

except KeyboardInterrupt: 
	print("Parou Server")
	server.stop()		