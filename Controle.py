import time
from PyQt5.QtCore import *
from PyQt5.QtGui import * 
from UDPCommunication.client import UDPClient

class Controle(QThread):
	def __init__(self,compVisual,sistBallAndPlate=None,remote=True):
		QThread.__init__(self)
		self.BallAndPlate = sistBallAndPlate
		self.CompVisual = compVisual
		self.remote = remote


		self.selectController = 0 # 0 - Manual, 1 - PID

		self.minAngle = -15
		self.maxAngle = +15

		self.setpoint = (0,0)
		self.time_prev = 0 # Utilizado para calcular o dT

		# Parametros pid
		self.kp = -0.5
		self.ki = 0
		self.kd = 0
		self.I = 0
		self.e_prev = 0

		if remote:
			self.client = UDPClient() # Importa comunicacao UDP caso seja remoto


	# def setClasses(self,compVisual):


	def setSetpoint(self,x,y):
		self.setpoint = (x,y)


	def Saturador(self,MV):
		if MV < self.minAngle:
			MV = self.minAngle

		elif MV > self.maxAngle:
			MV = self.maxAngle

		return MV

	def PIDController(self,measurement):
		# Retirado de https://softinery.com/blog/implementation-of-pid-controller-in-python/

		erro = setpoint - measurement

		P = self.kp * erro # Parametro proporcional
		tAtual = time.time()
		dt = tAtual - self.time_prev
		I = self.I + self.ki*erro*(dt)

		de = e - self.e_prev# Delta Erro
		D = self.kd * (de) / dT

		MV = P + I + D


		self.e_prev = erro
		self.time_prev = tAtual
		return MV


	def run(self): # runController
		self.ThreadActive = True
		while (self.ThreadActive):
			# print("Controlador ativado")
			# print(self.CompVisual.bolinha_coordenadas)
			if self.selectController == 1: 
				MV_x = PIDController(self.CompVisual.bolinha_coordenadas[0]) # PID no X
				MV_y = PIDController(self.CompVisual.bolinha_coordenadas[1]) # PID no X

				MV_x = self.Saturador(MV_x)
				MV_y = self.Saturador(MV_y)

				if not self.remote: # Caso a comunicacao seja local, enviar o sinal de controle diretamente
					self.BallAndPlate.setAngle(MV_x,MV_y)

				else:
					self.client.send(MV_x,MV_y)




	def stop(self):
		print("[DEBUG] Parou Controle")
		self.ThreadActive = False
		self.quit()