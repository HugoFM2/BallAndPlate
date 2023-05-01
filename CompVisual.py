import cv2 as cv
import numpy as np
import imutils
import mathFunctions as mf


from PyQt5.QtCore import *
from PyQt5.QtGui import * 


class CompVisual(QThread):
    print("[DEBUG] Thread CompVisual Inicializada")
    mainImage = pyqtSignal(np.ndarray)
    # cap = cv.VideoCapture(0) # Inicia devagar, solucao: https://answers.opencv.org/question/215586/why-does-videocapture1-take-so-long/
    cap = cv.VideoCapture(0, cv.CAP_DSHOW)
    print("[DEBUG] Camera Iniciada")

    pageSelect = 0 # 0 -> Ball Detection, 1 ->
    maskImage = pyqtSignal(np.ndarray)

    ArucoImage = pyqtSignal(np.ndarray)

    lowerHSV_Ball, upperHSV_Ball = (0,0,0), (0,0,0)
    erode_Ball, dilate_Ball = 0,0
    enableBallContour = False
    x_ball,y_ball,z_ball = pyqtSignal(int),pyqtSignal(int),pyqtSignal(int)

    enableUndistortion = True

    enableArUcoDetection = True
    enableChArUcoContour = False
    eulerAngles_Plate = pyqtSignal(dict)
    '''
    {"1" : [160,0,20],
     "2" : [160,0,20],
     "3" : [160,0,20]}
    '''
    # file1 = open("angleData.txt", "a") # Utilizado para filtrar - COMENTAR CASO NAO USE 

        

        

    def nothing(self,x):
        pass


    def saveToFile(self,fileName,data):
        import json
        with open(fileName, 'w', encoding='utf-8') as f:
            json.dump(data, f)


    def loadFromFile(self,filename):
        import json
        with open(filename, 'r') as f:
          data = json.load(f)
        return data


    def rotate_image(self,image, angle):
        """
        Rotate an image by a given angle in degrees.
        """
        height, width = image.shape[:2]
        image_center = (width/2, height/2)

        # Define the rotation matrix
        rotation_matrix = cv.getRotationMatrix2D(image_center, angle, 1.0)

        # Perform the rotation and return the rotated image
        rotated_image = cv.warpAffine(image, rotation_matrix, (width, height), flags=cv.INTER_LINEAR)

        return rotated_image

    def DetectaBolinha(self,hsv):

        kernel = np.ones((5, 5), np.uint8)

        # Criando a mascara para deteccao da bolinha
        mask = cv.inRange(hsv, self.lowerHSV_Ball, self.upperHSV_Ball)  # HSV Limits
        mask = cv.erode(mask, kernel, iterations=self.erode_Ball)       # Erode
        mask = cv.dilate(mask, kernel, iterations=self.dilate_Ball)     # Dilate

        if self.pageSelect == 0:
            self.maskImage.emit(cv.resize(mask, (420, 315)))
        
        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,
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
        return bolinha,bolinha_detectada

        
    def DetectChArUco(self,image,mtx,dist):
        image_Charuco = image.copy()
        dici_Aruco = { "ChArUco" : {    "rvec" : None,
                                        "tvec" : None}}

        ChArUcoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        ChArUcoBoard = cv.aruco.CharucoBoard((5, 5), 360, 300, ChArUcoDict) # Unidade está em 1/10mm, ou seja 360 = 36mm
        # CharucoParams = cv.aruco.CharucoParameters(mtx,dist)
        # print(CharucoParams)
        detectorParams = cv.aruco.DetectorParameters()


        detector = cv.aruco.CharucoDetector(ChArUcoBoard,detectorParams=detectorParams)


        arucoParams = cv.aruco.DetectorParameters()
       
        arucoDetector = cv.aruco.ArucoDetector(ChArUcoDict,arucoParams)
        (corners, ids, rejected) = arucoDetector.detectMarkers(image_Charuco)

        # Inicializando variaveis que retornam
        corrected_image_Charuco = cv.resize(image_Charuco.copy(),(384,384))
        rvec,tvec = None,None
        if ids is not None: # Se ao menos um marker foi detectado
            if self.enableChArUcoContour:
                image_Charuco = cv.aruco.drawDetectedMarkers(image_Charuco,corners)
            (retval,CharucoCorners,CharucoIds) = cv.aruco.interpolateCornersCharuco(corners,
                                                                  ids,image_Charuco,ChArUcoBoard,mtx,dist)
            if CharucoCorners is not None: # Se ao menos uma quina do charuco for detectada
                if self.enableChArUcoContour:
                    image_Charuco = cv.aruco.drawDetectedCornersCharuco(image_Charuco,CharucoCorners,CharucoIds)
                valid,rvec,tvec = cv.aruco.estimatePoseCharucoBoard(CharucoCorners,CharucoIds,ChArUcoBoard,mtx,dist,
                                                          None,None,False)
                
                if valid: # Detectou o board
                    if self.enableChArUcoContour:
                        image_Charuco = cv.drawFrameAxes(image_Charuco,mtx,dist,rvec,tvec,360,3) # Desenha os eixos


                    # Emite o angulo detectado para a GUI
                    dici_Aruco["ChArUco"]["rvec"] = rvec
                    self.eulerAngles_Plate.emit(dici_Aruco)


                    # Encontra o ponto no centro, localizado a 9cm do eixo superior esquerdo
                    ponto_centro = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=90,distanceY=90)


                    # image_Charuco = cv.circle(image_Charuco, tuple(ponto_centro), 10, (255, 255, 0), thickness=2)




                        

        # image_Charuco = cv.aruco.drawDetectedMarkers(image_Charuco,corners)
        # print(ids)
                    # Definindo os pontos do quadrado em 3d
                    ponto_SuperiorEsquerdo = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=0-50,distanceY=0-50.3) 
                    ponto_InferiorEsquerdo = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=0-50,distanceY=180+49.7) 
                    ponto_SuperiorDireito = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=180+50,distanceY=0-50.3)
                    ponto_InferiorDireito = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=180+50,distanceY=180+49.7) 
                    

                    pontos_src = np.array([ponto_SuperiorEsquerdo,ponto_InferiorEsquerdo,ponto_SuperiorDireito,ponto_InferiorDireito])

                    w,h = image_Charuco.shape[:2]

                    # Definindo os pontos que se deseja, ou seja, um quadrado 
                    dest_pts = np.array([[0,0],[0,384],[384,0],[384,384]])

                    homog, mask = cv.findHomography(pontos_src, dest_pts, 0)

                    # image_Charuco = cv.circle(image_Charuco, tuple(ponto_SuperiorEsquerdo), 10, (255, 0, 0), thickness=2)
                    # image_Charuco = cv.circle(image_Charuco, tuple(ponto_InferiorEsquerdo), 10, (0, 255, 0), thickness=2)
                    # image_Charuco = cv.circle(image_Charuco, tuple(ponto_SuperiorDireito), 10, (0, 0, 255), thickness=2)
                    # image_Charuco = cv.circle(image_Charuco, tuple(ponto_InferiorDireito), 10,  (255, 255, 0), thickness=2)
                      
                    corrected_image_Charuco = cv.warpPerspective(image, homog, (384, 384))
                    if self.pageSelect == 1:
                        # Desenhando o circulo externo, de 28cm
                        corrected_image_Charuco = cv.circle(corrected_image_Charuco, (int(384/2),int(384/2)), 190, (255, 255, 0), thickness=2)
                        

        return rvec,tvec,corrected_image_Charuco,image_Charuco
            
    def loadCameraCallibration(self): # Carrega os dados de calibração da camera, ret, cameraMatrix e DistCoeff 
        import json
        f = open("CallibrationData.json")
        data = json.load(f)
        # print
        return np.array(data['ret']), np.array(data['mtx']), np.array(data['dist'])        
        
    def run(self):
        self.ThreadActive = True

        ret, mtx, dist  = self.loadCameraCallibration() # Define os parametros intrínsecos da camera

        ret, img = self.cap.read()
        if ret:
            h, w, _ = img.shape


        while self.ThreadActive:
            ret, img = self.cap.read()
            if ret: # Caso detecte a camera
                success, img_orig = self.cap.read()
                h, w, _ = img_orig.shape # Height, width da imagem original da camera
                if self.enableUndistortion:
                    # newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
                    # img_orig = dst = cv.undistort(img, mtx, dist, None, newcameramtx)
                    img_orig = dst = cv.undistort(img, mtx, dist, None, None)


                gaussian_blurr = cv.GaussianBlur(img_orig.copy(), (3,3), cv.BORDER_DEFAULT) # Aplicando para reduzir noise and outliers
                hsv = cv.cvtColor(gaussian_blurr, cv.COLOR_BGR2HSV) # Aplica filtro HSV para detectar a bolinha


            if self.enableArUcoDetection: # Se a opcao de Detectar Aruco for Habilitada
                # img_orig = self.DetectAruco(img_orig,mtx,dist)
                rvec,tvec,corrected_image_Charuco,image_charuco = self.DetectChArUco(img_orig,mtx,dist)
                if rvec is not None: # Caso detecte o ChArUco
                    if self.enableChArUcoContour:
                        img_orig = image_charuco
                    gaussian_blurr__charuco = cv.GaussianBlur(corrected_image_Charuco.copy(), (3,3), cv.BORDER_DEFAULT) # Aplicando para reduzir noise and outliers
                    hsv_charuco = cv.cvtColor(gaussian_blurr__charuco, cv.COLOR_BGR2HSV)
                    
                    bolinha,bolinha_detectada = self.DetectaBolinha(hsv_charuco)
                    
                    if bolinha_detectada: # Caso bolinha seja detectada
                        x_bolinha,y_bolinha,r_bolinha,centro_bolinha = bolinha
                        self.x_ball.emit(x_bolinha)
                        self.y_ball.emit(y_bolinha)

                    if self.enableBallContour:
                        cv.circle(corrected_image_Charuco, (int(x_bolinha), int(y_bolinha)), int(r_bolinha), (0, 255, 255), 5)
                        cv.circle(corrected_image_Charuco, centro_bolinha, 5, (255,255 , 255), -1)


                    self.ArucoImage.emit(corrected_image_Charuco)
            

            self.mainImage.emit(img_orig)


    def stop(self):
        print("PAROU")
        self.ThreadActive = False
        self.quit()
        self.file1.close()
