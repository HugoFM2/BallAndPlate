import time
import numpy as np
# from PyQt5.QtCore import *
# from PyQt5.QtGui import * 


class Controle():
	def __init__(self,sistBallAndPlate=None):
		# QThread.__init__(self)
		self.BallAndPlate = sistBallAndPlate
		# self.CompVisual = compVisual
		# self.remote = remote


		self.selectController = 0 # 0 - Manual, 1 - PID

		self.minAngle = -20
		self.maxAngle = +20

		self.setpoint = (0,0)
		self.time_prev = [0,0] # Utilizado para calcular o dT

		self.currentAngle = (0,0)


		self.controllerHist = []
		self.setpointHist = [[],[]]

		self.posHist = [[],[]]
		self.mvHist = [[],[]]
		self.timeHist = []
		self.initTime = time.time() # Tempo de inicio do programa

		# Parametros pid

		self.kp = [0.3,0.3] # Kp x e y
		self.ki = [0.1,0.1]
		self.kd = [0.45,0.45]

		self.loadControlConfig('./ConfigFiles/ControlConfig.json') # Carrega o arquivo inicial com os parametros PIDs

		self.e_prev = [0,0]
		self.I = [0,0]




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
		# Implementar um Anti Windup?
		print(f"Setpoint{i}:{self.setpoint}")
		erro = setpoint - measurement


		P = self.kp[i] * erro # Parametro proporcional
		tAtual = time.time()
		dT = tAtual - self.time_prev[i]
		self.I[i] = self.I[i] + self.ki[i]*erro*(dT)


		# self.vi[i] = self.vi[i] + self.Ts*self.w[i]
		de = erro - self.e_prev[i]# Delta Erro

		if dT == 0:# Passar D como zero caso dT seja 0
			D = 0
		else:
			D = self.kd[i] * (de) / dT

		MV_PID = P + self.I[i] + D

		print(f"P:{P} / I:{self.I[i]} / D:{D}")
		self.e_prev[i] = erro
		self.time_prev[i] = tAtual
		# self.w[i] = -


		return MV_PID,erro




	def updateHist(self,bolinha_coordenadas,MV_x,MV_y):
		self.setpointHist[0].append(self.setpoint[0])
		self.setpointHist[1].append(self.setpoint[1])

		self.posHist[0].append(bolinha_coordenadas[0])
		self.posHist[1].append(bolinha_coordenadas[1])

		self.mvHist[0].append(-MV_y)
		self.mvHist[1].append(MV_x)

		self.timeHist.append(round(time.time() - self.initTime,2))
		# print(self.timeHist)

	def ClearHist(self):
		self.posHist = [[],[]]
		self.mvHist = [[],[]]
		self.timeHist = []

	def SaveHist(self,filename):
		import pandas as pd
		print("Salvando Arquivo")
		dictt = {'time': self.timeHist,
		         'x': self.posHist[0],
		         'y': self.posHist[1],
		         'MV_x' : self.mvHist[0],
		         'MV_y' : self.mvHist[1]}
		         
		# print(dictt)
		df = pd.DataFrame(dictt)
		df.to_csv(filename)

	def loadControlConfig(self,filename):
		"""
		Load Control calibration variables from json
		"""
		import json
		f = open(filename)
		data = json.load(f)
		self.kp = data['Kp']
		self.ki = data['Ki']
		self.kd = data['Kd']

	def saveControlConfig(self,fileName):
		import json
		data = {"Kp" : self.kp,
				"Ki" : self.ki,
				"Kd" : self.kd}

		with open(fileName, 'w', encoding='utf-8') as f:
			json.dump(data, f)





	def runController(self,bolinha_coordenadas): # runController

		self.e_prev_x = 0
		self.e_prev_y = 0

		
		self.measurement = (bolinha_coordenadas[0],bolinha_coordenadas[1])

		MV_x = 0
		MV_y = 0
		if(self.selectController == 0):# Zerar PID caso mude para manual
			self.e_prev = [0,0]
			self.I = [0,0]
			self.time_prev = [time.time(),time.time()]
	


		if (self.measurement[0] == 100): # Caso a bolinha caia, desativar o controle
			print("CONTROLE DESATIVADO - BOLINHA CAIU")
			# self.e_prev = [0,0]
			# self.I = [0,0]
			self.time_prev = [time.time(),time.time()]
			return

		# print("Controlador ativado")
		# print(bolinha_coordenadas)
		elif self.selectController == 1: 
			# Teste trajetoria circular
			# r = 10
			# omega = 0.5
			# self.setpoint = ( r*np.cos(omega*time.time()),r*np.sin(omega*time.time()) )
			


			print(self.measurement)
			MV_x,erro_x = self.PIDController(self.setpoint[0],self.measurement[0],0) # PID no X
			MV_y,erro_y = self.PIDController(self.setpoint[1],self.measurement[1],1) # PID no Y

			print("MV_X,MV_Y:", MV_x,MV_y)
			MV_x = self.Saturador(MV_x)
			MV_y = self.Saturador(MV_y)

			print("MV_X,MV_Y:", MV_x,MV_y)
			# print("Measurement:", self.measurement[0],self.measurement[1])
			self.BallAndPlate.setAngle(-MV_y,MV_x)


		# time.sleep(0.02) # Delay de 20ms para combinar com a taxa de atualizacao do servo (50Hz)

		# Atualiza os dados
		self.updateHist(bolinha_coordenadas,MV_x,MV_y)



	def stop(self):
		print("[DEBUG] Parou Controle")