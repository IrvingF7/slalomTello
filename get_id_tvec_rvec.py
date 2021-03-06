import os
import numpy as np
import cv2
import cv2.aruco as aruco
import time


def calibrate():

    cap = cv.VideoCapture(0)
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    # checkerboard of size (9 x 7) is used
    objp = np.zeros((7*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:7].T.reshape(-1,2)

    # arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # resizing for faster detection
        frame = cv.resize(frame, (640, 480))
        # using a greyscale picture, also for faster detection
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv.findChessboardCorners(gray, (9,7), None)

        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and display the corners
            cv.drawChessboardCorners(frame, (9,7), corners, ret)
            #write_name = 'corners_found'+str(idx)+'.jpg'

        # Display the resulting frame
        cv.imshow('Calibration',frame)
        if cv.waitKey(100) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()
    cv.waitKey(10)
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    #create a file to store data
    from lxml import etree
    from lxml.builder import E

    global fname
    with open(fname, "w") as f:
        f.write("{'ret':"+str(ret)+", 'mtx':"+str(list(mtx))+', "dist":'+str(list(dist))+'}')
        f.close()



#%%
#test wheater already calibrated or not
path = os.path.abspath('..')
fname = path + "\\slalomTello\\res\\calibration_parameters.txt"
print(fname)

try:
    f = open(fname, "r")
    f.read()
    f.close()
except:
    calibrate()

#%%

cap = cv.VideoCapture(0)

#importing aruco dictionary
dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_250)

#calibration parameters
f = open(fname, "r")
ff = [i for i in f.readlines()]
f.close()
from numpy import array
parameters = eval(''.join(ff))
mtx = array(parameters['mtx'])
dist = array(parameters['dist'])

# Create absolute path from this module
file_abspath = os.path.join(os.path.dirname(__file__), 'Samples/box.obj')

tvec = [[[0, 0, 0]]]
rvec = [[[0, 0, 0]]]

aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_250)
markerLength = 0.25   # Here, our measurement unit is centimetre.
parameters = cv.aruco.DetectorParameters_create()
parameters.adaptiveThreshConstant = 10

while True:

    ret, frame = cap.read()
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    font = cv.FONT_HERSHEY_SIMPLEX
    if np.all(ids != None):
        rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)

        #print(ids)
        #print(corners)
        #print(rvec)

        for i in range(0, ids.size):
            cv.aruco.drawAxis(frame, mtx, dist, rvec[i], tvec[i], 0.1)

            # show translation vector on the corner
            font = cv.FONT_HERSHEY_SIMPLEX
            text = str([round(i,5) for i in tvec[i][0]])
            position = tuple(corners[i][0][0])
            cv.putText(frame, text, position, font, 0.4, (0, 0, 0), 1, cv.LINE_AA)

            #get tvec, rvec of each id
            print('ids: ', ids[i])
            print('translation: ', tvec[i][0])
            print('rotation: ', rvec[i][0])
            #print('distance: ', np.linalg.norm(tvec[i][0]))
        cv.aruco.drawDetectedMarkers(frame, corners)
    else:
        tvec = [[[0, 0, 0]]]
        rvec = [[[0, 0, 0]]]
    cv.imshow('frame',frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
