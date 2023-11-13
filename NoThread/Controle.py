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

		self.minAngle = -10
		self.maxAngle = +10

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

		# Paremtros LQR
		self.ang_prev = [0,0]
		self.p_prev = [0,0]

		#Parametros Observador
		self.x_h_prev = np.matrix([[0],[0],[0],[0]])
		self.y_h_prev = np.matrix([[0],[0],[0],[0]])




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
		erro = setpoint - measurement


		P = self.kp[i] * erro # Parametro proporcional
		tAtual = time.time()
		dT = tAtual - self.time_prev[i]
		I = self.I[i] + self.ki[i]*erro*(dT)


		# self.vi[i] = self.vi[i] + self.Ts*self.w[i]
		de = erro - self.e_prev[i]# Delta Erro

		if dT == 0:# Passar D como zero caso dT seja 0
			D = 0
		else:
			D = self.kd[i] * (de) / dT

		MV_PID = P + D + I


		self.e_prev[i] = erro
		self.time_prev[i] = tAtual
		# self.w[i] = -


		return MV_PID,erro

	def ObservadorLuenberger(self,u,x_h,y):
		'''
		X_hat(k+1) = Ax(k) + Bu(k) + L[y(k) - Cx(k)]
		sendo L o ganho do observador
		Referencia: hhttps://github.com/FBaldo98/LuenbergerObserver/blob/master/src/luenberger/luenberger_observer.c
		'''

		# Transforma os valores em matrizes 1x1
		u = np.matrix(u)
		y = np.matrix(y)

		A = np.matrix([[0,1,0,0],
					   [0,0,-5.886,0],
					   [0,0,0,1],
					   [0,0,0,0]])

		B = np.matrix([[0],[0],[0],[0]])

		C = np.matrix([[1,0,0,0]])

		L = np.matrix([
						[4.6315],
						[10.2254],
						[-2.1153],
						[-1.0]])

		# Calcula (A-L*C)
		A_x_h = A@x_h

		B_u = B@u

		L_y = L@y
		L_cx = L@C@x_h


		X_h_k1 = A_x_h + B_u + L_y - L_cx


		return X_h_k1




	def LQRController(self,setpoint,measurement,x_h,i):
		''' PARAMETROS QUE EU PRECISO TER (Obtenho a partir do calculo com o passo anterior)
		x
		v_x
		alfa -> PRECISO PEGAR O ALFA/BETA
		v_alfa
		'''
		k = [-24.1469, -18.5755, 42.3216, 9.468, 2.2361]

		erro = setpoint[i] - measurement

		# Para facilitar a leitura
		# pos_atual = setpoint[i] 
		# pos_atual = measurement
		# ang_atual = angle[i]


		# Calculo do dT para calcular velocidades
		# tAtual = time.time()
		# dT = tAtual - self.time_prev
		# if dT == 0:
		# 	vel_ang = 0
		# 	vel_pos = 0
		# else:
		# 	print("VEL_ANG:",(ang_atual - self.ang_prev[i]))
		# 	print("dT:",dT)
			# vel_ang = (ang_atual - self.ang_prev[i])/dT
			# vel_pos = (pos_atual - self.p_prev[i])/dT # Delta P / deltaT

		# Parametros com observador
		pos_atual,vel_pos,ang_atual,vel_ang = x_h
		# vel_pos = 
		# k[0]*pos_atual
		# k[1]*vel_pos
		# k[2]*alfa
		# k[3]*vel_ang
		# k[4]*e # ERRO

		# vetor deitado, vezes um em pé, ai precisaria só somar
		MV_LQR = - (k[0]*pos_atual + k[1]*vel_pos + k[2]*ang_atual +k[3]*vel_ang + k[4]*erro) # usar o menos caso a acao seja contraria

		# a IDENTIDADE R JA esta passando direto na soma do k[4]*e

		# Definindo vetores anteriores
		# self.time_prev = tAtual
		# self.p_prev[i] = measurement
		# self.ang_prev[i] = angle

		return MV_LQR,erro


	def updateHist(self,bolinha_coordenadas):
		self.setpointHist[0].append(self.setpoint[0])
		self.setpointHist[1].append(self.setpoint[1])

		self.posHist[0].append(bolinha_coordenadas[0])
		self.posHist[1].append(bolinha_coordenadas[1])

		self.timeHist.append(round(time.time() - self.initTime,2))
		# print(self.timeHist)


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


	def SaveHist(self,filename):
		import pandas as pd
		print("Salvando Arquivo")
		dictt = {'time': self.timeHist, 'x': self.posHist[0], 'y': self.posHist[1]}
		print(dictt)
		df = pd.DataFrame(dictt)
		df.to_csv(filename)


	def runController(self,bolinha_coordenadas): # runController

		self.e_prev_x = 0
		self.e_prev_y = 0

		self.updateHist(bolinha_coordenadas)
		self.measurement = (bolinha_coordenadas[0],bolinha_coordenadas[1])


		if(self.selectController == 0):# Zerar PID caso mude para manual
			self.e_prev = [0,0]
			self.I = [0,0]

			self.x_h_prev = [self.measurement[0],0,0,0] # Assume que a bolinha esta em uma posicao x e inclinacao 0
			self.y_h_prev = [self.measurement[1],0,0,0]
			


			
		
		if (self.measurement[0] == 100): # Caso a bolinha caia, desativar o controle
			print("CONTROLE DESATIVADO - BOLINHA CAIU")
			self.e_prev = [0,0]
			self.I = [0,0]
			return
		# print("Controlador ativado")
		# print(bolinha_coordenadas)
		elif self.selectController == 1: 
			print(self.measurement)
			MV_x,erro_x = self.PIDController(self.setpoint[0],self.measurement[0],0) # PID no X
			MV_y,erro_y = self.PIDController(self.setpoint[1],self.measurement[1],1) # PID no Y

			MV_x = self.Saturador(MV_x)
			MV_y = self.Saturador(MV_y)
			print("MV_X,MV_Y:", MV_x,MV_y)
			print("Measurement:", self.measurement[0],self.measurement[1])
			self.BallAndPlate.setAngle(-MV_y,MV_x)


		elif self.selectController == 2: # Controlador LQR
			print(self.measurement)
			print("x_h_prev",self.x_h_prev)
			# currentAngle = (np.deg2rad(self.BallAndPlate.Plate.xAngle),np.deg2rad(self.BallAndPlate.Plate.yAngle)) # Angulos em radianos

			MV_x,erro_x = self.LQRController(self.setpoint,self.measurement[0],x_h=self.x_h_prev,i=0) # LQR no X
			MV_y,erro_y = self.LQRController(self.setpoint,self.measurement[1],x_h=self.y_h_prev,i=1) # LQR no Y


			self.x_h_prev = self.ObservadorLuenberger(u=MV_x,x_h=self.x_h_prev ,y=self.measurement[0]) # Observador em X
			
			self.y_h_prev = self.ObservadorLuenberger(u=MV_y,x_h=self.y_h_prev ,y=self.measurement[1]) # Observador em X

			print("MV_X,MV_Y:", MV_x,MV_y)
			self.BallAndPlate.setAngle(MV_y,MV_x)

		# time.sleep(0.02) # Delay de 20ms para combinar com a taxa de atualizacao do servo (50Hz)




	def stop(self):
		print("[DEBUG] Parou Controle")