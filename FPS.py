# import the necessary packages
import time
class FPS:
	def __init__(self):
		# store the start time, end time, and total number of frames
		# that were examined between the start and end intervals
		self._prevTime = 0
		self._actTime = 0
		self._numFrames = 0

	def updateActTime(self):
		# start the timer
		self._actTime = time.time()



	def CalculateFPS(self):
		fps = int(1/(self._actTime-self._prevTime))
		self._prevTime = self._actTime
		return fps