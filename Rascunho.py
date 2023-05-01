import cv2
import mathFunctions as mf
from scipy.spatial.transform import Rotation as R
import numpy as np
rvec=[1,1,2]
ponto = [2,2,2]
res =mf.TranslatePoint(ponto,rvec,distanceX=0)


print(res)

rotMat = R.from_rotvec(np.array(rvec).flatten()).as_matrix()
res2 = np.matmul(ponto,rotMat) # Ponto nas novas coordenadas
print(f"res2: {res2}")
res2 = res2 + [0,0,1]
print(f"res2: {res2}")
print(cv2.__version__)