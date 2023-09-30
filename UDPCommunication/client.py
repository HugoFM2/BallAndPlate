import socket
import time

# import the necessary packages
from threading import Thread
import cv2
class UDPClient:
	def __init__(self):
		self.UDP_IP = "192.168.15.234" # listen to everything
		self.UDP_PORT = 12345 # port

		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		# self.sock.bind((UDP_IP, UDP_PORT))


		
	def send(self,MV_x,MV_y):
		# start the thread to read frames from the video stream
		self.sock.sendto( bytes(f'{MV_x};{MV_y}','utf-8'), (self.UDP_IP, self.UDP_PORT))




if __name__ == "__main__":
	client = UDPClient()
	a = 0
	b = 1
	try:
		while True:
			a+=1
			b+=1
			client.send(a,b)
			# print("enviou")
			time.sleep(1)
	except KeyboardInterrupt:

		print("Parou Client")
