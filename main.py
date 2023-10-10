# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, QtWidgets
from CompVisual import CompVisual
from Controle import Controle

from CinematicaInversa.BallAndPlate import BallAndPlate
from CinematicaInversa.Servo import Servo
from CinematicaInversa.Plate import Plate

import cv2 as cv
import numpy as np
from GUI.integrado import Ui_MainWindow
import mathFunctions as mf




from numpy import sin,cos,deg2rad



class ControlMainWindow(QtWidgets.QMainWindow):
	def __init__(self, remote=False, parent=None):
		super(ControlMainWindow, self).__init__(parent)
		self.ui = Ui_MainWindow()
		self.ui.setupUi(self)
		self.remote = remote # Definindo se a compVisual será remota(Na raspberry pi ou comandada remotamente por UDP)
		self.setupUi()
		

		self.ui.BallSelectButton.clicked.connect(lambda : self.enableSelectPage(0))
		self.ui.ArUcoSelectButton.clicked.connect(lambda : self.enableSelectPage(1))
		self.ui.SurfaceSelectButton.clicked.connect(lambda : self.enableSelectPage(2))
		self.ui.OtherOptionsSelectButton.clicked.connect(lambda : self.enableSelectPage(3))

		#Options for surface DetectionPage
		self.ui.ManualMode_Surface.clicked.connect(lambda:self.enableSelectSurfacePage(0))
		self.ui.AutoMode_Surface.clicked.connect(lambda:self.enableSelectSurfacePage(1))

		#Options for control selection
		self.ui.ControlType_Off.clicked.connect(lambda:self.enableControlType(0))
		self.ui.ControlType_PID.clicked.connect(lambda:self.enableControlType(1))
		self.ui.ControlType_LQR.clicked.connect(lambda:self.enableControlType(2))



		self.ui.enableBallContour.stateChanged.connect(self.CheckBallContour)
		self.ui.enableChArUcoContour.stateChanged.connect(self.CheckChArucoContour)

		self.ui.enableUndistortion.stateChanged.connect(self.CheckUndistortion)
		self.ui.enableArUcoDetection.stateChanged.connect(self.CheckArUcoDetection)

		# Ball Page Options
		self.ui.Slider_H_Ball_min.valueChanged['int'].connect(self.setBallConfiguration)
		self.ui.Slider_S_Ball_min.valueChanged['int'].connect(self.setBallConfiguration)
		self.ui.Slider_V_Ball_min.valueChanged['int'].connect(self.setBallConfiguration)

		self.ui.Slider_H_Ball_max.valueChanged['int'].connect(self.setBallConfiguration)
		self.ui.Slider_S_Ball_max.valueChanged['int'].connect(self.setBallConfiguration)
		self.ui.Slider_V_Ball_max.valueChanged['int'].connect(self.setBallConfiguration)

		self.ui.Slider_Erode_Ball.valueChanged.connect(self.setBallConfiguration)
		self.ui.Slider_Dilate_Ball.valueChanged.connect(self.setBallConfiguration)

		#Botao de definir manualmente o angulo da superficie
		self.ui.SendManualSurfaceSetpoint.clicked.connect(self.setManualAngles)

		# Botao de setar parametros PID
		self.ui.SetPIDValues.clicked.connect(self.setPIDParameters)

		#Save and load Button
		self.ui.Load_Ball_Button.clicked.connect(lambda : self.LoadBallConfiguration())
		self.ui.Save_Ball_Button.clicked.connect(lambda : self.SaveBallConfiguration())
		print("[DEBUG] Programa Iniciado")

		# Save historical data
		self.ui.saveHistData_Button.clicked.connect(lambda : self.SaveHistData())

	def setupUi(self):
		self.compVisual = CompVisual()
		

		self.compVisual.start()
		self.compVisual.mainImage.connect(self.ImageUpdateSlot)
		self.compVisual.maskImage.connect(self.BallMaskUpdate)
		self.compVisual.ArucoImage.connect(self.ArucoImageUpdate)

		self.compVisual.x_ball.connect(self.UpdateXValue)
		self.compVisual.y_ball.connect(self.UpdateYValue)
		self.compVisual.z_ball.connect(self.UpdateZValue)

		#Conecta valores de yaw pitch e roll
		self.compVisual.eulerAngles_Plate.connect(self.UpdatePlateAngles)

		self.enableSelectPage(0) # Define a pagina inicial como ball selector
		self.enableSelectSurfacePage(0) # Define a pagina inicial como ball selector
		self.LoadBallConfiguration() # Carrega o arquivo ballConfiguration

		# Setar valores iniciais das checkboxs
		self.ui.enableArUcoDetection.setChecked(True)
		self.ui.enableUndistortion.setChecked(True)

		# #CinematicaInversa
		if self.remote:
			Servo1 = Servo(servoIndex=22,zeroAngle=1,servoPos=np.matrix([[0,10,0]]).T,remote=True)
			Servo2 = Servo(servoIndex=1,zeroAngle=0,servoPos=np.matrix([[10*sin(deg2rad(120)),10*cos(deg2rad(120)),0]]).T,remote=True)
			Servo3 = Servo(servoIndex=2,zeroAngle=0,servoPos=np.matrix([[10*sin(deg2rad(240)),10*cos(deg2rad(240)),0]]).T,remote=True)
			plate = Plate(height=11)



		else:

			from adafruit_servokit import ServoKit    #https://circuitpython.readthedocs.io/projects/servokit/en/latest/
			pca = ServoKit(channels=16)
			Servo1 = Servo(servoIndex=22,zeroAngle=1,servoPos=np.matrix([[0,10,0]]).T,pca=pca)
			Servo1.pca.servo[Servo1.servoIndex].set_pulse_width_range(750,2150)
			Servo2 = Servo(servoIndex=1,zeroAngle=10,servoPos=np.matrix([[10*sin(deg2rad(120)),10*cos(deg2rad(120)),0]]).T,pca=pca)
			Servo3 = Servo(servoIndex=2,zeroAngle=12,servoPos=np.matrix([[10*sin(deg2rad(240)),10*cos(deg2rad(240)),0]]).T,pca=pca)

			plate = Plate(height=11)

		self.UpdateZValue(plate.height)

		self.BallAndPlate = BallAndPlate(Servo1,Servo2,Servo3,plate,remote=True,remoteType="Serial")

		#Controle
		self.Controle = Controle(self.compVisual,self.BallAndPlate)
		self.Controle.start()



	def setPIDParameters(self):
		self.Controle.kp = self.ui.KpValue.value()
		self.Controle.ki = self.ui.KiValue.value()
		self.Controle.kd = self.ui.KdValue.value()

			
	def setManualAngles(self):
		angle_x = self.ui.alphaManualAngleSetpoint.value()
		angle_y = self.ui.betaManualAngleSetpoint.value()
		self.ui.AlfaValue.setText(str(angle_x))
		self.ui.BetaValue.setText(str(angle_y))
		self.BallAndPlate.setAngle(angle_x,angle_y)


	def CheckBallContour(self):
		self.compVisual.enableBallContour = self.ui.enableBallContour.isChecked()

	def CheckChArucoContour(self):
		self.compVisual.enableChArUcoContour = self.ui.enableChArUcoContour.isChecked()

	def enableSelectPage(self,indexPage):
		self.ui.stackedWidget.setCurrentIndex(indexPage)
		self.compVisual.pageSelect = indexPage

	def enableControlType(self,num):
		self.Controle.selectController = num
		if num == 1:
			print("Controle PID Habilitado!")

	def enableSelectSurfacePage(self,indexPage):
		self.ui.JanelasModoSurface.setCurrentIndex(indexPage)

	def CheckArUcoDetection(self):
		self.compVisual.enableArUcoDetection = self.ui.enableArUcoDetection.isChecked()

	def CheckUndistortion(self):
		self.compVisual.enableUndistortion = self.ui.enableUndistortion.isChecked()

	def setBallConfiguration(self):
		# print(valor)
		self.ui.H_Ball_min_label.setText(str(self.ui.Slider_H_Ball_min.value()))
		# self.ui.H_Ball_min_label.setText = 45
		self.ui.S_Ball_min_label.setText(str(self.ui.Slider_S_Ball_min.value()))
		self.ui.V_Ball_min_label.setText(str(self.ui.Slider_V_Ball_min.value()))
		self.compVisual.lowerHSV_Ball = (self.ui.Slider_H_Ball_min.value(),self.ui.Slider_S_Ball_min.value(),self.ui.Slider_V_Ball_min.value())

		self.ui.H_Ball_max_label.setText(str(self.ui.Slider_H_Ball_max.value()))
		self.ui.S_Ball_max_label.setText(str(self.ui.Slider_S_Ball_max.value()))
		self.ui.V_Ball_max_label.setText(str(self.ui.Slider_V_Ball_max.value()))
		self.compVisual.upperHSV_Ball = (self.ui.Slider_H_Ball_max.value(),self.ui.Slider_S_Ball_max.value(),self.ui.Slider_V_Ball_max.value())

		self.ui.Erode_Ball_label.setText(str(self.ui.Slider_Erode_Ball.value()))
		self.compVisual.erode_Ball = self.ui.Slider_Erode_Ball.value()

		self.ui.Dilate_Ball_label.setText(str(self.ui.Slider_Dilate_Ball.value()))
		self.compVisual.dilate_Ball = self.ui.Slider_Dilate_Ball.value()

	def ImageUpdateSlot(self, Image):
		qt_img = self.convert_cv_qt(Image)
		self.ui.disp_main.setPixmap(qt_img)

	def BallMaskUpdate(self,Image):
		qt_img = self.convert_cv_qt(Image)
		self.ui.disp_Ball.setPixmap(qt_img)

	def ArucoImageUpdate(self,Image):
		qt_img = self.convert_cv_qt(Image)
		self.ui.disp_Aruco.setPixmap(qt_img)

	def UpdateXValue(self,x_value):
		# print(x_value)
		self.ui.Qlabel_posX.setText(str(x_value))

	def UpdateYValue(self,y_value):
		# print(y_value)
		self.ui.Qlabel_posY.setText(str(y_value))

	def UpdateZValue(self,z_value):
		print(z_value)
		self.ui.Qlabel_posZ.setText(str(z_value))

	def UpdatePlateAngles(self,dici):
		# print("aa")
		ChArUcoAngles = mf.rVecToEulerList(dici["ChArUco"]["rvec"])
		# euler_ID0 = Rotation.from_rotvec(np.array(dici["0"]["rvec"])).as_euler('xyz',degrees=True)
		self.ui.Yaw_Angle.setText(f'{ChArUcoAngles[0]:6.2f}')
		self.ui.Pitch_Angle.setText(f'{ChArUcoAngles[1]:6.2f}')
		self.ui.Roll_Angle.setText(f'{ChArUcoAngles[2]:6.2f}')
  

	def convert_cv_qt(self, cv_img):
		"""Convert from an opencv image to QPixmap"""
		rgb_image = cv.cvtColor(cv_img, cv.COLOR_BGR2RGB)
		h, w, ch = rgb_image.shape
		bytes_per_line = ch * w
		convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
		# p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
		return QtGui.QPixmap.fromImage(convert_to_Qt_format)

	def LoadBallConfiguration(self):
		data = self.compVisual.loadFromFile('./ConfigFiles/ballConfiguration.json')
		lower_HSV = data["Lower HSV"]
		upper_HSV = data["Upper HSV"]
		self.ui.Slider_H_Ball_min.setValue(lower_HSV[0])
		self.ui.Slider_S_Ball_min.setValue(lower_HSV[1])
		self.ui.Slider_V_Ball_min.setValue(lower_HSV[2])

		self.ui.Slider_H_Ball_max.setValue(upper_HSV[0])
		self.ui.Slider_S_Ball_max.setValue(upper_HSV[1])
		self.ui.Slider_V_Ball_max.setValue(upper_HSV[2])

		self.ui.Slider_Erode_Ball.setValue(data["Erode"])
		self.ui.Slider_Dilate_Ball.setValue(data["Dilate"])
		self.setBallConfiguration() # Seta os valores na GUI

	def SaveBallConfiguration(self):
		data = {}
		data["Lower HSV"] = list(self.compVisual.lowerHSV_Ball)
		data["Upper HSV"] = list(self.compVisual.upperHSV_Ball)
		data["Erode"] = self.compVisual.erode_Ball
		data["Dilate"] = self.compVisual.dilate_Ball

		self.compVisual.saveToFile('ballConfiguration.json',data)

	def SaveHistData(self):
		
		self.Controle.SaveHist("OUT.csv")

if __name__ == "__main__":
	import sys,os, time
	os.system('"pyuic5 -x GUI\\Integrado.ui -o GUI\\integrado.py') # Compila a GUI (Nao compila de primeira, é necessario executar 2x)
	app = QtWidgets.QApplication(sys.argv)
	# MainWindow = QtWidgets.ControlMainWindow()
	# ui = Ui_MainWindow()
	# ui.setupUi(MainWindow)

	mySW = ControlMainWindow(remote=True)
	mySW.show()
	# MainWindow.show()
	sys.exit(app.exec_())
