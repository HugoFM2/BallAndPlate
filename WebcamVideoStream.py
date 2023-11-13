# import the necessary packages
from threading import Thread
import cv2 as cv
class WebcamVideoStream:
	def __init__(self, src=0):
		# initialize the video camera stream and read the first frame
		# from the stream
		self.stream = cv.VideoCapture(src, cv.CAP_DSHOW)
		self.stream.set(cv.CV_CAP_PROP_FRAME_WIDTH, 640)
		self.stream.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

		(self.grabbed, self.frame) = self.stream.read()
		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False
	
		
	def start(self):
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		return self


	def update(self):
		# keep looping infinitely until the thread is stopped
		while True:
			# if the thread indicator variable is set, stop the thread
			if self.stopped:
				return
			# otherwise, read the next frame from the stream
			(self.grabbed, self.frame) = self.stream.read()
	def read(self):
		# return the frame most recently read
		return self.frame
	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True


if __name__ == '__main__':
	from imutils.video import WebcamVideoStream
	vs = WebcamVideoStream(src=1).start()
	while True:
		img_orig = vs.read()
		cv.imshow("aaa", img_orig) 
		  
		if cv.waitKey(1) == 27: 
			break  # esc to quit
