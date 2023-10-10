import serial 
import time 



class SerialComm():
	def __init__(self,port):
		self.arduino = serial.Serial(port=port, baudrate=115200, timeout=0.1,write_timeout = 0.1,rtscts=False, dsrdtr=False,xonxoff=False) 
		# self.initialTime = time.time()
		# self.vezes = 0


	def send(self,Servo1Angle,Servo2Angle,Servo3Angle):
		# print(f"Enviado Serial: {round(Servo1Angle,2)};{round(Servo2Angle,2)};{round(Servo3Angle,2)}F")
		self.arduino.flush()
		data = bytes(f'{round(Servo1Angle,2)};{round(Servo2Angle,2)};{round(Servo3Angle,2)}F','utf-8')
		self.arduino.write(data) 
		# print(len(data))
		
		# self.vezes +=1
		# print(f"Duração: {time.time() - self.initialTime} / Vezes: {self.vezes}")
		# time.sleep(0.01) 
