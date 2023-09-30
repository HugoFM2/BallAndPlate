# import the necessary packages
from threading import Thread
import cv2 as cv
import numpy as np
import imutils
import mathFunctions as mf
import time
import matplotlib.pyplot as plt
import json

from imutils.video import WebcamVideoStream
from FPS import FPS

class CompVisual:
	def __init__(self):
		# initialize the variable used to indicate if the thread should
		# be stopped
		self.stopped = False
		self.fps = FPS()

		#CharUcoDefinition parameters
		self.ArUcoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
		# self.ChArUcoBoard = cv.aruco.CharucoBoard((5, 5), 360, 300, self.ArUcoDict) # Unidade está em 1/10mm, ou seja 360 = 36mm
		self.ChArUcoBoard = cv.aruco.CharucoBoard((3, 3), 600, 500, self.ArUcoDict) # Para o Caso de DiamondBoard
		# CharucoParams = cv.aruco.CharucoParameters(mtx,dist)
		# print(CharucoParams)
		self.detectorParams = cv.aruco.DetectorParameters()
		# detectorParams.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX # High cpu usage
		self.detectorParams.cornerRefinementMethod = cv.aruco.CORNER_REFINE_NONE
		self.detector = cv.aruco.CharucoDetector(self.ChArUcoBoard,detectorParams=self.detectorParams)

		self.arucoParams = cv.aruco.DetectorParameters()   
		self.enableChArUcoContour = True

		self.arucoDetector = cv.aruco.ArucoDetector(self.ArUcoDict,self.arucoParams)

		# CharucoDiamond parameters
		tag = np.zeros((1800, 1800, 1), dtype="uint8")
		self.DiamondBoard = cv.aruco.drawCharucoDiamond(self.ArUcoDict, (1,2,3,4), 600,500, tag, 1)


		# Ball parameters
		self.lowerHSV_Ball, self.upperHSV_Ball = (0,0,0), (0,0,0)
		self.erode_Ball, dilate_Ball = 0,0
		self.enableBallContour = False
		self.bolinha_coordenadas = (0,0)
		self.bolinha_detectada = False

		#Load undistortion parameters
		f = open("./ConfigFiles/CallibrationDataRPI.json")
		data = json.load(f)		
		self.ret = np.array(data['ret'])
		self.mtx = np.array(data['mtx'])
		self.dist = np.array(data['dist'])

	def start(self):
		# Start the thread
		Thread(target=self.run, args=()).start()
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

	def undistortImage(self):
		"""
		Load calibration variables from camera (ret,CameraMatrix and DistCoeffs) and return the map from undistortion
		"""
		map1,map2 = cv.initUndistortRectifyMap(self.mtx, self.dist, None, None, (640,480),
				cv.CV_16SC2) # Getting the undistortion map

		return map1, map2


	def DetectaBolinha(self,image):
		"""
		Detect the ball in a HSV image (giving a lower and upper global HSV variables), and return if the ball is detected
		and its coordinates
		"""
		gaussian_blurr__charuco = cv.GaussianBlur(image.copy(), (3,3), cv.BORDER_DEFAULT) # Aplicando para reduzir noise and outliers
		hsv = cv.cvtColor(gaussian_blurr__charuco, cv.COLOR_BGR2HSV)

		kernel = np.ones((5, 5), np.uint8)

		# Criando a mascara para deteccao da bolinha
		mask = cv.inRange(hsv, self.lowerHSV_Ball, self.upperHSV_Ball)  # HSV Limits
		mask = cv.erode(mask, kernel, iterations=2)       # Erode
		mask = cv.dilate(mask, kernel, iterations=2)     # Dilate


		
		cnts = cv.findContours(mask, cv.RETR_EXTERNAL,
								cv.CHAIN_APPROX_SIMPLE)

		cnts = imutils.grab_contours(cnts)
		center = None
		x,y,radius = None, None, None
		bolinha_detectada = False
		if len(cnts) > 0:
			c = max(cnts, key=cv.contourArea)
			((x, y), radius) = cv.minEnclosingCircle(c)
			M = cv.moments(c)
			if(M["m00"] != 0):
				center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
				bolinha_detectada = True

				
		bolinha = (x,y,radius,center)



		return bolinha_detectada,bolinha,mask

	def DetectChArUco(self,image):
		"""
		Detect the charuco board and return its rvec(Rotation Vector) tvec (Translation vector, assuming the upper-left corner),
		and 2 images
		"""
		image_Charuco = image.copy()
		dici_Aruco = { "ChArUco" : {    "rvec" : None,
										"tvec" : None}}
	   
		
		(corners, ids, rejected) = self.arucoDetector.detectMarkers(image_Charuco)

		# Inicializando variaveis que retornam
		corrected_image_Charuco = cv.resize(image_Charuco,(600,600)) # A cada 2 pixels, 1 centimetro
		rvec,tvec = None,None
		if ids is not None: # Se ao menos um marker foi detectado
			if self.enableChArUcoContour:
				image_Charuco = cv.aruco.drawDetectedMarkers(image_Charuco,corners)
			(retval,CharucoCorners,CharucoIds) = cv.aruco.interpolateCornersCharuco(corners,
																  ids,image_Charuco,self.ChArUcoBoard,self.mtx,self.dist)

			if CharucoCorners is not None: # Se ao menos uma quina do charuco for detectada
				if self.enableChArUcoContour:
					image_Charuco = cv.aruco.drawDetectedCornersCharuco(image_Charuco,CharucoCorners,CharucoIds)
				valid,rvec,tvec = cv.aruco.estimatePoseCharucoBoard(CharucoCorners,CharucoIds,self.ChArUcoBoard,self.mtx,self.dist,
														  None,None,False)
				
				if valid: # Detectou o board
					if self.enableChArUcoContour:
						image_Charuco = cv.drawFrameAxes(image_Charuco,self.mtx,self.dist,rvec,tvec,360,3) # Desenha os eixos


					# # Emite o angulo detectado para a GUI
					# dici_Aruco["ChArUco"]["rvec"] = rvec
					# self.eulerAngles_Plate.emit(dici_Aruco)


					# # Encontra o ponto no centro, localizado a 9cm do eixo superior esquerdo
					# ponto_centro = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=90,distanceY=90)
					# print(f'Ponto Centro: {ponto_centro}')

					# image_Charuco = cv.circle(image_Charuco, tuple(ponto_centro), 10, (255, 255, 0), thickness=2)
						

					# # Definindo os pontos do quadrado em 3d
					# ponto_SuperiorEsquerdo = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=0-50,distanceY=0-50.3) 
					# ponto_InferiorEsquerdo = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=0-50,distanceY=180+49.7) 
					# ponto_SuperiorDireito = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=180+50,distanceY=0-50.3)
					# ponto_InferiorDireito = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=180+50,distanceY=180+49.7) 
					# pontos_src = np.array([ponto_SuperiorEsquerdo,ponto_InferiorEsquerdo,ponto_SuperiorDireito,ponto_InferiorDireito])

					# w,h = image_Charuco.shape[:2]

					# # Definindo os pontos que se deseja, ou seja, um quadrado de 600x600 
					# dest_pts = np.array([[0,0],[0,600],[600,0],[600,600]])

					# homog, mask = cv.findHomography(pontos_src, dest_pts, 0)

					  
					# corrected_image_Charuco = cv.warpPerspective(image, homog, (600, 600))
					# if self.enableChArUcoContour:
					# 	# Desenhando o circulo externo, de 28cm
					# 	corrected_image_Charuco = cv.circle(corrected_image_Charuco, (int(600/2),int(600/2)), 300, (255, 255, 0), thickness=2)
						
					# 	# Desenhando o centro da superficie
					# 	corrected_image_Charuco = cv.circle(corrected_image_Charuco, (int(600/2),int(600/2)), 2, (255, 255, 0), thickness=2)
					
					
		return rvec,tvec,image_Charuco


	def DetectDiamond(self,image):
		image_Charuco = image.copy()
		(corners, ids, rejected) = self.arucoDetector.detectMarkers(image_Charuco)
		if ids is not None: # Se ao menos um marker foi detectado
			print("Teste")
			diamondCorners,diamondIds = cv.aruco.detectCharucoDiamond(image_Charuco,corners,ids,1.2) # 1.2 is ratio between square and tag
			# diamond_corners, diamond_ids, marker_corners, marker_ids
			print(diamondCorners,diamondIds)
			# image_Charuco = cv.aruco.drawDetectedDiamonds(image_Charuco,diamondCorners,diamondIds)

		return 1,1,image_Charuco

	def run(self):
		vs = WebcamVideoStream(src=0).start()
		map1,map2 = self.undistortImage()

		while True:
			if self.stopped:
				return
			self.fps.updateActTime() # Atualiza FPS

			imagem = vs.read()

			#Aplicar correcao de distorcao
			imagem = cv.remap(imagem,map1,map2,cv.INTER_LINEAR)
			# cv.imshow("aa",imagem)


			# Habilitando detectorAruco
			# rvec, tvec, imagem_charuco = self.DetectChArUco(imagem)

			# #HabilitandoDetectorDiamond
			# rvec, tvec, imagem_charuco = self.DetectDiamond(imagem)

			#Detectando a bolinha
			# bolinha_detectada,bolinha,mask = self.DetectaBolinha(imagem)


			#Calcula FPS
			print(f'FPS:{self.fps.CalculateFPS()}')
			cv.imshow("bb",imagem)
			cv.waitKey(1)

			# # DetectCharuco
			# rvec,tvec,corrected_image_Charuco = self.DetectChArUco(imagem,mtx,dist)
			# if rvec is not None and False: # Caso detecte o ChArUco
			# 	bolinha_detectada,bolinha = self.DetectaBolinha(imagem)
			# 	if bolinha_detectada: # Caso bolinha seja detectada
			#         x_bolinha,y_bolinha,r_bolinha,centro_bolinha = bolinha
			#         print(centro_bolinha)


