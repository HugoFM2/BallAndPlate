import time
from PyQt5.QtCore import *
from PyQt5.QtGui import * 


class Controle(QThread):
	def __init__(self,compVisual,sistBallAndPlate=None):
		QThread.__init__(self)
		self.BallAndPlate = sistBallAndPlate
		self.CompVisual = compVisual
		# self.remote = remote


		self.selectController = 0 # 0 - Manual, 1 - PID

		self.minAngle = -10
		self.maxAngle = +10

		self.setpoint = (0,0)
		self.time_prev = 0 # Utilizado para calcular o dT


		self.controllerHist = []
		self.setpointHist = [[],[]]

		self.posHist = [[],[]]
		self.mvHist = [[],[]]
		self.timeHist = []
		self.initTime = time.time() # Tempo de inicio do programa

		# Parametros pid
		self.kp = 0.3
		self.ki = 0.05
		self.kd = 0.45
		# self.kp = -0.2
		# self.ki = -0
		# self.kd = -0.05
		self.e_prev = [0,0]
		self.I = [0,0]


	# def setClasses(self,compVisual):


	def setSetpoint(self,x,y):
		self.setpoint = (x,y)


	def Saturador(self,MV):
		if MV < self.minAngle:
			MV = self.minAngle

		elif MV > self.maxAngle:
			MV = self.maxAngle

		return MV

	def PIDController(self,setpoint,measurement,i):
		# Retirado de https://softinery.com/blog/implementation-of-pid-controller-in-python/
		# Implementar um Anti Windup
		erro = setpoint - measurement


		P = self.kp * erro # Parametro proporcional
		tAtual = time.time()
		dT = tAtual - self.time_prev
		I = self.I[i] + self.ki*erro*(dT)


		# self.vi[i] = self.vi[i] + self.Ts*self.w[i]
		de = erro - self.e_prev[i]# Delta Erro

		if dT == 0:# Passar D como zero caso dT seja 0
			D = 0
		else:
			D = self.kd * (de) / dT

		MV_PID = P + D + I


		self.e_prev[i] = erro
		self.time_prev = tAtual
		# self.w[i] = -


		return MV_PID,erro

	def updateHist(self):
		self.setpointHist[0].append(self.setpoint[0])
		self.setpointHist[1].append(self.setpoint[1])

		self.posHist[0].append(self.CompVisual.bolinha_coordenadas[0])
		self.posHist[1].append(self.CompVisual.bolinha_coordenadas[1])

		self.timeHist.append(round(time.time() - self.initTime,2))
		# print(self.timeHist)

	def SaveHist(self,filename):
		import pandas as pd
		print("Salvando Arquivo")
		dictt = {'time': self.timeHist, 'x': self.posHist[0], 'y': self.posHist[1]}
		print(dictt)
		df = pd.DataFrame(dictt)
		df.to_csv(filename)

	def run(self): # runController
		self.ThreadActive = True
		self.e_prev_x = 0
		self.e_prev_y = 0
		while (self.ThreadActive):
			self.updateHist()
			if(self.selectController == 0):# Zerar PID caso mude para manual
				self.e_prev
				self.I
				
			self.measurement = (self.CompVisual.bolinha_coordenadas[0],self.CompVisual.bolinha_coordenadas[1])
			if (self.measurement[0] == 100): # Caso a bolinha caia, desativar o controle
				time.sleep(0.0001)
				continue
			# print("Controlador ativado")
			# print(self.CompVisual.bolinha_coordenadas)
			elif self.selectController == 1: 
				print(self.measurement)
				MV_x,erro_x = self.PIDController(self.setpoint[0],self.measurement[0],0) # PID no X
				MV_y,erro_y = self.PIDController(self.setpoint[1],self.measurement[1],1) # PID no X

				MV_x = self.Saturador(MV_x)
				MV_y = self.Saturador(MV_y)


				self.BallAndPlate.setAngle(MV_y,MV_x)

			

			time.sleep(0.02) # Delay de 20ms para combinar com a taxa de atualizacao do servo (50Hz)




	def stop(self):
		print("[DEBUG] Parou Controle")
		self.ThreadActive = False
		self.quit()