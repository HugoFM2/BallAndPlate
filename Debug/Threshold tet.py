import cv2 as cv
import numpy as np
import json

cap = cv.VideoCapture(0, cv.CAP_DSHOW)
ret, img = cap.read()

ChArUcoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
ChArUcoBoard = cv.aruco.CharucoBoard((5, 5), 360, 300, ChArUcoDict)




def on_change(val):
	global threshconstant
	threshconstant = val

def onChange_MaxThreshold(val):
	global adaptiveThreshMaxValue
	adaptiveThreshMaxValue = val

# Load camera calibration
f = open('../CallibrationDataRPI.json')
data = json.load(f)
ret = data['ret']
mtx = data['mtx']
dist = data['dist']

#Create random image
img = np.zeros((300,512,3), np.uint8)
cv.namedWindow('trackbars')
#Create taskbar

cv.createTrackbar('threshConstant', "trackbars", 7, 100, on_change)
cv.createTrackbar('MaxValue', "trackbars", 23, 100, onChange_MaxThreshold)

# Valores que podem ser alterados
threshconstant = 7
adaptiveThreshMaxValue = 23


if ret:
	h, w, _ = img.shape


	while True:
		ret, img = cap.read()

		detectorParams = cv.aruco.DetectorParameters()
		detectorParams.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
		detectorParams.adaptiveThreshConstant = threshconstant
		detectorParams.adaptiveThreshWinSizeMax = adaptiveThreshMaxValue

		detector = cv.aruco.CharucoDetector(ChArUcoBoard,detectorParams=detectorParams)
		arucoParams = cv.aruco.DetectorParameters()
		# Filtros
		kernel = np.ones((5,5),np.float32)/10
		# dst = cv.medianBlur(img,5)
		dst = cv.bilateralFilter(img,9,75,75)


		arucoDetector = cv.aruco.ArucoDetector(ChArUcoDict,arucoParams)
		(corners, ids, rejected) = arucoDetector.detectMarkers(img)

		if ids is not None:
			img = cv.aruco.drawDetectedMarkers(img,corners)

		(corners_f, ids_f, rejected) = arucoDetector.detectMarkers(dst)
		if ids is not None:
			dst = cv.aruco.drawDetectedMarkers(dst,corners_f)

		cv.imshow("Webcam Edited", img)


		cv.imshow("Filtered Image", dst)
		retval, img_threshold = cv.threshold(img,threshconstant,adaptiveThreshMaxValue,cv.THRESH_BINARY)
		cv.imshow("Threshold original", img_threshold)
		retval, img_threshold_tiltered = cv.threshold(dst,threshconstant,adaptiveThreshMaxValue,cv.THRESH_BINARY)
		cv.imshow("Threshold Filtered", img_threshold_tiltered)
		




		pressedKey = cv.waitKey(25) & 0xFF
		if pressedKey == ord('q'):
			break	