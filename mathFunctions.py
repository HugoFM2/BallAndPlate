import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import cv2 as cv

# def getAverage(lista,m=2):
# 	# https://www.kdnuggets.com/2017/02/removing-outliers-standard-deviation-python.html
# 	lista = np.array(lista)
# 	d = np.abs(lista - np.median(lista))
# 	mdev = np.median(d)
# 	s = d/mdev if mdev else np.zeros(len(d))
# 	avg = np.mean(lista[s<m])
# 	return avg



def rotateMatrix(rotVec,axis,degrees):
	#Bug, precisa colocar a metade do angulo para funcionar
	r1 = R.from_rotvec(np.array(rotVec).flatten()).as_rotvec()
	r2 = R.from_euler(axis, degrees, degrees=True).as_matrix()
	res = np.matmul(r2,r1)
	return res

def TranslatePoint(ponto,rotVec,distanceX=0, distanceY=0,distanceZ=0):
    # Move o ponto em um novo eixo, definido pelo rotVec
    #https://www.brainvoyager.com/bv/doc/UsersGuide/CoordsAndTransforms/SpatialTransformationMatrices.html
    rotMat = R.from_rotvec(np.array(rotVec).flatten()).as_matrix()

    # Construindo Transformation Matrix
    T = np.eye(4)
    T[0:3, 0:3] = rotMat # O primeiro quadrado 3x3 corresponde a rotation matrix
    # A parte superior direita corresponde ao quanto deseja transladar nesse vetor
    T[0,3] = distanceX
    T[1,3] = distanceY
    T[2,3] = distanceZ
    # print(f"rotMat:\n {rotMat}\n - T: \n{T}")
    #Transformando o ponto3d em uma matriz 4x1 para realizar a multiplicacao com a transformation Matrix
    ponto_4x1 = np.hstack((ponto,1)).reshape(4,1)
    # print(ponto_4x1)

    ponto_trasladado = np.matmul(T,ponto_4x1)[:3] # Só pegar os 3 primeiros dados, ou seja, os pontos x,y e z
    return ponto_trasladado





def rVecToEulerList(rvec):
	if rvec is not None:
		rvec = np.array(rvec).flatten()
		rvec = R.from_rotvec(rvec)
		return rvec.as_euler('xyz',degrees=True)
	else:
		return [0,0,0]


def convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=0, distanceY=0,distanceZ=0):
    ponto_3d = tvec.flatten()

    # Por algum motivo, 900000 = 9cm, entao para facilitar, sera feita uma conversao para mm, para o argumento ser em mm
    point_transformed = TranslatePoint(ponto_3d,rvec,distanceX=distanceX*10000,distanceY=distanceY*10000,distanceZ=distanceZ*10000) 


    ponto_2d,_ = cv.projectPoints(point_transformed/1000,rvec,tvec,mtx,dist)
    ponto_2d = ponto_2d.flatten().astype(int)
    return ponto_2d

