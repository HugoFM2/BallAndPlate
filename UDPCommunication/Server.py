import socket, time



# import the necessary packages
from threading import Thread
import cv2
class UDPServer:
	def __init__(self, src=0):
		self.UDP_IP = "0.0.0.0" # listen to everything
		self.UDP_PORT = 12345 # port

		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		

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
		self.sock.settimeout(2)
		self.sock.bind((self.UDP_IP, self.UDP_PORT))
		
		while not self.stopped:
			try:
				# self.data, addr = self.sock.recvfrom(512) # random buffer size, doesn't matter here..
				self.data = self.sock.recv(512) #random buffer size, doesn't matter here..
				# print(f"receive data:{self.data}")
				self.data = [int(x) for x in self.data.decode('utf-8').split(";")]
				# print("received message:", self.data)
			except socket.timeout:
				continue




	def read(self):
		# return the frame most recently read
		return self.data


	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True


if __name__ == "__main__":
	try:
		server = UDPServer()
		server.start()
		while True:
			print(server.data)
			time.sleep(0.1)

	except KeyboardInterrupt: 
		print("Parou Server")
		server.stop()		
