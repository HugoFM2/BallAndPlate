import socket
import time

# import the necessary packages
from threading import Thread
import cv2
class UDPClient:
	def __init__(self):
		self.UDP_IP = "localhost" # listen to everything
		self.UDP_PORT = 12345 # port

		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		# self.sock.bind((UDP_IP, UDP_PORT))

		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False
		self.data = None
		
	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.run, args=()).start()
		return self

	def run(self):
		# keep looping infinitely until the thread is stopped
		while True:
			print("Turn ON")
			self.sock.sendto(b'20;10;5', (self.UDP_IP, self.UDP_PORT))
			time.sleep(2)
			print("Turn OFF")
			self.sock.sendto(b'0;0;0', (self.UDP_IP, self.UDP_PORT))
			time.sleep(2)

			
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				return
			# otherwise, read the next frame from the stream


	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True


if __name__ == "__main__":
	client = UDPClient()
	client.start()
	try:
		while True:
			time.sleep(1)
	except KeyboardInterrupt:

		print("Parou Client")
		client.stop()
