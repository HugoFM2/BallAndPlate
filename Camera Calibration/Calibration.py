import cv2 as cv
import numpy as np
import os

def calibrateCamera(allCharucoCorners,allCharucoIds,ChArUcoBoard,ImgSize):
    retval, cameraMatrix, distCoeffs, rvecs, tvecs = cv.aruco.calibrateCameraCharuco(
        charucoCorners= allCharucoCorners,
        charucoIds=allCharucoIds,
        board=ChArUcoBoard,
        imageSize=ImgSize,
        cameraMatrix=None,
        distCoeffs=None
                                               )
    return retval, cameraMatrix, distCoeffs, rvecs, tvecs

def saveCalibrateToFile(fileName,retVal,cameraMatrix,distCoeffs):
    data = {"ret": retVal, 
            "mtx": cameraMatrix.tolist(),
            "dist" : distCoeffs.tolist()}

    fname = "CallibrationData.json"
    import json
    with open(fname, "w") as f:
        json.dump(data, f, indent=4)


ChArUcoDict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
ChArUcoBoard = cv.aruco.CharucoBoard((5, 5), 360, 300, ChArUcoDict)
allCharucoCorners = []
allIdsCharuco = []
allImages = []

cap = cv.VideoCapture(0, cv.CAP_DSHOW)
ret, img = cap.read()


if ret:
    h, w, _ = img.shape


    while True:
        validCandidate = False

        ret, img = cap.read()
        

        imgEdited = img.copy()

        arucoParams = cv.aruco.DetectorParameters()
        arucoDetector = cv.aruco.ArucoDetector(ChArUcoDict,arucoParams)
        (corners, ids, rejected) = arucoDetector.detectMarkers(imgEdited)

        #Interpolate charuco corners
        if ids is not None:
            imgEdited = cv.aruco.drawDetectedMarkers(imgEdited,corners)
            (retval,CharucoCorners,CharucoIds) = cv.aruco.interpolateCornersCharuco(corners,
                                                                  ids,img,ChArUcoBoard,minMarkers=2)
            # print(corners)
            if CharucoCorners is not None:
                imgEdited = cv.aruco.drawDetectedCornersCharuco(imgEdited,CharucoCorners)
                if len(ids) > 4:
                    validCandidate = True
                    ImgSize = img.shape[:2]

        # imgEdited = cv.putText(imgEdited,"Press 'a' to add or 'c' to initiate calibration. 'ESC' to finish and calibrate",
                # (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        

        # cv.imshow("Webcam", img)
        cv.imshow("Webcam Edited", imgEdited)

        pressedKey = cv.waitKey(25) & 0xFF
        if pressedKey == ord('q'):
          break


        elif pressedKey == ord('a'):
            print("tecla A pressionada! Capturando imagem")
            if validCandidate: # Caso a imagem a ser capturada detecte um corner, salvar lista
                allCharucoCorners.append(CharucoCorners)
                allIdsCharuco.append(CharucoIds)
                allImages.append(img)

        elif pressedKey == ord('c'):
            print("Tecla C pressionada! Iniciando calibração e salvando")
            if len(allIdsCharuco) < 5:
                print(f"Pontos não suficientes, existem apenas {len(allIdsCharuco)} pontos")
            else:
                retval, cameraMatrix, distCoeffs, rvecs, tvecs = calibrateCamera(allCharucoCorners,allIdsCharuco,
                    ChArUcoBoard,ImgSize)
                saveCalibrateToFile("Calibration.json",retval, cameraMatrix, distCoeffs)
                print("Arquivo Salvo!")


