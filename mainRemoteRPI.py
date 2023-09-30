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
	Servo1 = Servo(pca,servoIndex=0,zeroAngle=14,servoPos=np.matrix([[0,10,0]]).T)
	Servo2 = Servo(pca,servoIndex=1,zeroAngle=10,servoPos=np.matrix([[10*sin(deg2rad(120)),10*cos(deg2rad(120)),0]]).T)
	Servo3 = Servo(pca,servoIndex=2,zeroAngle=12,servoPos=np.matrix([[10*sin(deg2rad(240)),10*cos(deg2rad(240)),0]]).T)
	plate = Plate()

	ballAndPlate = BallAndPlate(Servo1,Servo2,Servo3,plate)

	server = UDPServer()
	server.start()
	while True:
		
		if server.data is not None:
			x,y = server.data
			if x > 15:
				x = 15
			if y > 15:
				y = 15
			ballAndPlate.setAngle(x,y)
			print(f'AngleX:{x} / AngleY: {y}')
		else:
			ballAndPlate.setAngle(0,0)
		time.sleep(0.1)

except KeyboardInterrupt: 
	print("Parou Server")
	server.stop()		