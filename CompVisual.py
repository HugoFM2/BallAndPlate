import cv2 as cv
import numpy as np
import imutils
import mathFunctions as mf
import time
import matplotlib.pyplot as plt
from imutils.video import WebcamVideoStream


from PyQt5.QtCore import *
from PyQt5.QtGui import * 


class CompVisual(QThread):
    print("[DEBUG] Thread CompVisual Inicializada")
    mainImage = pyqtSignal(np.ndarray)
    # cap = cv.VideoCapture(-1) # Inicia devagar, solucao: https://answers.opencv.org/question/215586/why-does-videocapture1-take-so-long/
    # cap = cv.VideoCapture(0, cv.CAP_DSHOW)

    #Setting resolution
    # cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    # cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    # cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    #CharUcoDefinition parameters
    ArUcoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    ChArUcoBoard = cv.aruco.CharucoBoard((5, 5), 360, 300, ArUcoDict) # Unidade está em 1/10mm, ou seja 360 = 36mm
    # ChArUcoBoard = cv.aruco.CharucoBoard((3, 3), 600, 500, ChArUcoDict,np.array([2,3,4,5])) # Para o Caso de DiamondBoard
    # CharucoParams = cv.aruco.CharucoParameters(mtx,dist)
    # print(CharucoParams)
    detectorParams = cv.aruco.DetectorParameters()
    # detectorParams.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX # High cpu usage
    detectorParams.cornerRefinementMethod = cv.aruco.CORNER_REFINE_NONE

    detector = cv.aruco.CharucoDetector(ChArUcoBoard,detectorParams=detectorParams)

    #Detector Aruco
    marker_size=500 # in 1/10mm
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

    arucoParams = cv.aruco.DetectorParameters()    

    #Calculate FPS
    prev_frame_time = 0
    new_frame_time = 0
    font = cv.FONT_HERSHEY_SIMPLEX

    print("[DEBUG] Camera Iniciada")

    pageSelect = 0 # 0 -> Ball Detection, 1 ->
    maskImage = pyqtSignal(np.ndarray)

    ArucoImage = pyqtSignal(np.ndarray)

    lowerHSV_Ball, upperHSV_Ball = (0,0,0), (0,0,0)
    erode_Ball, dilate_Ball = 0,0
    enableBallContour = False
    x_ball,y_ball,z_ball = pyqtSignal(float),pyqtSignal(float),pyqtSignal(float)

    enableUndistortion = True

    enableArUcoDetection = True
    enableChArUcoContour = False
    eulerAngles_Plate = pyqtSignal(dict)
    # file1 = open("angleData.txt", "a") # Utilizado para filtrar - COMENTAR CASO NAO USE 

    bolinha_coordenadas = (0,0)
    bolinha_detectada = False


        

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
        """
        Detect the ball in a HSV image (giving a lower and upper global HSV variables), and return if the ball is detected
        and its coordinates
        """
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
            bolinha_detectada = True

            # M = cv.moments(c)
            # if(M["m00"] != 0):
            #     center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            #     bolinha_detectada = True

                
        bolinha = (x,y,radius)



        return bolinha_detectada,bolinha

    def DetectSingleAruco(self,image,corners,ids,idAruco,arucoDetector,mtx,dist):
        # print(ids)
        if [idAruco] in ids:
            idx = np.where(ids==[12])[0][0] # Index do Aruco de ID idAruco
            # print(corners[idx])
            _,rvec,tvec = cv.solvePnP(self.marker_points,corners[idx],mtx,dist)
            # image = cv.drawFrameAxes(image,mtx,dist,rvec,tvec,500,3) # Desenha os eixos
            rmat,_ = cv.Rodrigues(rvec)
            
            rmat = mf.rotate3d(rmat,angle_x=np.deg2rad(0),angle_y=np.deg2rad(0),angle_z=np.deg2rad(0))
            # rmat = np.linalg.inv(rmat)
            # print(rmat)
            # print('________')
            rvec,_= cv.Rodrigues(rmat)
            # print(rvec)
            image = cv.drawFrameAxes(image,mtx,dist,rvec,tvec,500,3) # Desenha os eixos
            # print(rvec)
            return rvec,tvec
        return None,[0]


    def DetectChArUco(self,image,mtx,dist):
        """
        Detect the charuco board and return its rvec(Rotation Vector) tvec (Translation vector, assuming the upper-left corner),
        and 2 images
        """
        image_Charuco = image.copy()
        dici_Aruco = { "ChArUco" : {    "rvec" : None,
                                        "tvec" : None}}


        arucoDetector = cv.aruco.ArucoDetector(self.ArUcoDict,self.arucoParams)
        (corners, ids, rejected) = arucoDetector.detectMarkers(image_Charuco)
        if ids is None:
            ids = [] 
        # Inicializando variaveis que retornam
        corrected_image_Charuco = cv.resize(image_Charuco.copy(),(600,600)) # A cada 2 pixels, 1 centimetro
        rvec,tvec = None,None
        if len(ids) > 3: # Se ao menos 3 marker foi detectado
            # print(ids)
            # rvecMesa,tvecMesa = self.DetectSingleAruco(image_Charuco,corners,ids,idAruco=12,arucoDetector=arucoDetector,mtx=mtx,dist=dist)
            if self.enableChArUcoContour:
                # print("detecotu marker")
                image_Charuco = cv.aruco.drawDetectedMarkers(image_Charuco,corners)
            (retval,CharucoCorners,CharucoIds) = cv.aruco.interpolateCornersCharuco(corners,
                                                                  ids,image_Charuco,self.ChArUcoBoard,mtx,dist)

            if CharucoCorners is not None: # Se ao menos uma quina do charuco for detectada

                if self.enableChArUcoContour:
                    image_Charuco = cv.aruco.drawDetectedCornersCharuco(image_Charuco,CharucoCorners,CharucoIds)


                valid,rvec,tvec = cv.aruco.estimatePoseCharucoBoard(CharucoCorners,CharucoIds,self.ChArUcoBoard,mtx,dist,
                                                          None,None,False)
                # if rvecMesa is not None:
                #     print(mf.rVecToEulerList(rvec-rvecMesa))

                
                if valid: # Detectou o board
                    if self.enableChArUcoContour:
                        image_Charuco = cv.drawFrameAxes(image_Charuco,mtx,dist,rvec,tvec,360,3) # Desenha os eixos


                    # Emite o angulo detectado para a GUI
                    dici_Aruco["ChArUco"]["rvec"] = rvec
                    self.eulerAngles_Plate.emit(dici_Aruco)


                    # Encontra o ponto no centro, localizado a 9cm do eixo superior esquerdo
                    ponto_centro = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=90,distanceY=90)
                    # print(f'Ponto Centro: {ponto_centro}')

                    image_Charuco = cv.circle(image_Charuco, tuple(ponto_centro), 10, (255, 255, 0), thickness=2)
                        

                    # Definindo os pontos do quadrado em 3d
                    ponto_SuperiorEsquerdo = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=0-50,distanceY=0-50.3) 
                    ponto_InferiorEsquerdo = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=0-50,distanceY=180+49.7) 
                    ponto_SuperiorDireito = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=180+50,distanceY=0-50.3)
                    ponto_InferiorDireito = mf.convert3DPointTo2DAndTranslate(rvec,tvec,mtx,dist,distanceX=180+50,distanceY=180+49.7) 
                    pontos_src = np.array([ponto_SuperiorEsquerdo,ponto_InferiorEsquerdo,ponto_SuperiorDireito,ponto_InferiorDireito])

                    w,h = image_Charuco.shape[:2]

                    # Definindo os pontos que se deseja, ou seja, um quadrado de 600x600 
                    dest_pts = np.array([[0,0],[0,600],[600,0],[600,600]])

                    homog, mask = cv.findHomography(pontos_src, dest_pts, 0)

                    # image_Charuco = cv.circle(image_Charuco, tuple(ponto_SuperiorEsquerdo), 10, (255, 0, 0), thickness=2)
                    # image_Charuco = cv.circle(image_Charuco, tuple(ponto_InferiorEsquerdo), 10, (0, 255, 0), thickness=2)
                    # image_Charuco = cv.circle(image_Charuco, tuple(ponto_SuperiorDireito), 10, (0, 0, 255), thickness=2)
                    # image_Charuco = cv.circle(image_Charuco, tuple(ponto_InferiorDireito), 10,  (255, 255, 0), thickness=2)
                      
                    corrected_image_Charuco = cv.warpPerspective(image, homog, (600, 600))
                    if self.pageSelect == 1:
                        # Desenhando o circulo externo, de 28cm
                        corrected_image_Charuco = cv.circle(corrected_image_Charuco, (int(600/2),int(600/2)), 300, (255, 255, 0), thickness=2)
                        
                        # Desenhando o centro da superficie
                        corrected_image_Charuco = cv.circle(corrected_image_Charuco, (int(600/2),int(600/2)), 2, (255, 255, 0), thickness=2)
                    
                    

        return rvec,tvec,corrected_image_Charuco,image_Charuco
            

    def loadCameraCallibration(self,filename):
        """
        Load calibration variables from camera (ret,CameraMatrix and DistCoeffs)
        """
        import json
        f = open(filename)
        data = json.load(f)

        return np.array(data['ret']), np.array(data['mtx']), np.array(data['dist'])        
        
    def CalculateFPS(self,image):
        # Calculate FPS
        self.new_frame_time = time.time()
        deltaT = (self.new_frame_time-self.prev_frame_time)
        fps="0"
        if deltaT != 0:
            fps = str(int( 1/(self.new_frame_time-self.prev_frame_time) ))
        self.prev_frame_time = self.new_frame_time
        cv.putText(image, fps, (7, 70), self.font, 3, (100, 255, 0), 3, cv.LINE_AA)
        return image

    def run(self):
        vs = WebcamVideoStream(src=0).start()
        self.ThreadActive = True
        # ret, mtx, dist  = self.loadCameraCallibration("CallibrationDataRPI.json") # Define os parametros intrínsecos da camera RPI
        ret, mtx, dist  = self.loadCameraCallibration("./ConfigFiles/CallibrationDataC270.json") # Define os parametros intrínsecos da camera C270
        # ret, img = self.cap.read()
        # if ret:
        #     h, w, _ = img.shape

        map1, map2 = cv.initUndistortRectifyMap(mtx, dist, None, None, (640,480),
                cv.CV_16SC2) # Getting the undistortion map

        while self.ThreadActive:
            # print(time.time())
            img_orig = vs.read()

            if True:


                h, w, _ = img_orig.shape # Height, width da imagem original da camera

                # Aplica Correção de distorção
                if self.enableUndistortion:
                    img_orig = cv.remap(img_orig,map1,map2,cv.INTER_LINEAR)


                # Aplica detecção dos charucos
                if self.enableArUcoDetection: # Se a opcao de Detectar Aruco for Habilitada

 
                    rvec,tvec,corrected_image_Charuco,image_charuco = self.DetectChArUco(img_orig,mtx,dist)

                    if rvec is not None: # Caso detecte o ChArUco
                        if self.enableChArUcoContour:
                            img_orig = image_charuco

                        gaussian_blurr__charuco = cv.GaussianBlur(corrected_image_Charuco.copy(), (3,3), cv.BORDER_DEFAULT) # Aplicando para reduzir noise and outliers
                        hsv_charuco = cv.cvtColor(gaussian_blurr__charuco, cv.COLOR_BGR2HSV)

                        self.bolinha_detectada,bolinha = self.DetectaBolinha(hsv_charuco)
                        
                        if self.bolinha_detectada: # Caso bolinha seja detectada
                            x_bolinha,y_bolinha,r_bolinha = bolinha
                            # print(centro_bolinha)

                            # Como a imagem possui 600x600, redimensionar a posicao da bolinha para atender esses requisitos
                            # Modificando as coordenadas para o centro da superficie ser o ponto (0,0)
                            x_bolinha_coord = (x_bolinha/2 - 150)/10 # Divide por 10 para ser em metros
                            y_bolinha_coord = (y_bolinha/2 - 150)/10 # Divide por 10 para ser em metros
                            if (x_bolinha_coord**2 + y_bolinha_coord**2) < 22500: # Caso a Bolinha esteja no Raio

                                self.bolinha_coordenadas = (x_bolinha_coord,y_bolinha_coord) # Divide por 100 para ficar em metros
                                self.x_ball.emit(round(float(x_bolinha_coord/10),2))
                                self.y_ball.emit(round(float(y_bolinha_coord/10),2))

                            else:
                                self.bolinha_coordenadas = (100,100)
                                self.x_ball.emit(100.0)
                                self.y_ball.emit(100.0)

                            if self.enableBallContour:
                                cv.circle(corrected_image_Charuco, (int(x_bolinha), int(y_bolinha)), int(r_bolinha), (0, 255, 255), 5)
                                # print(centro_bolinha)
                                cv.circle(corrected_image_Charuco, (centro_bolinha), 5, (255,255 , 255), -1)

                        else: # Caso a bolinha não seja detectada, enviar um valor fixo para detecção que a bolinha nao esta la
                            self.bolinha_coordenadas = [100,100]
                            self.x_ball.emit(100)
                            self.y_ball.emit(100) 

                        self.ArucoImage.emit(cv.resize(corrected_image_Charuco.copy(),(384,384))) # Redimensiona a imagem para o tamanho da GUI)
                

                # Calculate FPS
                self.CalculateFPS(img_orig)


                self.mainImage.emit(img_orig)



    def stop(self):
        print("PAROU")
        self.ThreadActive = False
        self.quit()
        self.file1.close()
