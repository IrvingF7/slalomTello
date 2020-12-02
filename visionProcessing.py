import numpy as np
import cv2
import pdb
import datetime
from numpy import array
import matplotlib.pyplot as plt

def recordingSetup(filename, cap):
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    return cv2.VideoWriter(filename + '.avi',fourcc, fps, (width,height))

def loadCalibration():
    f = open("calibrationData2.txt", "r")
    ff = [i for i in f.readlines()]
    f.close()
    calibration_data = eval(''.join(ff))
    mtx = np.array(calibration_data['mtx'])
    dist = np.array(calibration_data['dist'])
    return mtx, dist

def arucoAnalysis(arucoDict, frame, parameters, mtx, dist, textRecorder):
    # Detect and draw the markers in the image
    markerCorners, markerIDs, rejectedCandidates = cv2.aruco.detectMarkers(frame, arucoDict, parameters=parameters)
    cv2.aruco.drawDetectedMarkers(frame, markerCorners, markerIDs)

    # Detect marker pose
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 0.05, mtx, dist)
    print(rvec)
    # Scaling up translation for ease of reading
    try:
        tvec = tvec*100
    except:
        x = 3;

    if np.all(markerIDs != None):
        for marker_idx in range(0,len(markerIDs)):
            # Draw rotation axes and display translation
            cv2.aruco.drawAxis(frame, mtx, dist, rvec[marker_idx], tvec[marker_idx], 5)
            font = cv2.FONT_HERSHEY_SIMPLEX
            text = str([round(pos,2) for pos in tvec[marker_idx][0]])
            position = tuple(markerCorners[marker_idx][0][0])
            cv2.putText(frame, text, position, font, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

            # Record data
            textRecorder.write(str([rvec[marker_idx][0].tolist(), tvec[marker_idx][0].tolist(), markerIDs[marker_idx][0]]))
            if marker_idx < len(markerIDs)-1:
                textRecorder.write(',')
    else:
            textRecorder.write(str([rvec, tvec]))
    textRecorder.write("\n")

    return frame, markerIDs, rvec, tvec

def visionProcessing():
    # Read input from camera or recording
        # Webcam = 0
        # Tello is "udp://@0.0.0.0:11111"
        # Recording is "<filename>"
    camera = 0
    cap = cv2.VideoCapture(camera)
    scale = 3


    # Setup for writing data
    filename = "Outputs/output_" + datetime.datetime.now().strftime("%m-%d-%Y_%H%M%S")
    print(filename)
    vidRecorder = recordingSetup(filename, cap)
    textRecorder = open(filename + ".txt",'a',buffering=1)

    #Load the dictionary that was used to generate the markers.
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

    # Initialize the aruco corner detector parameters using default values
    parameters =  cv2.aruco.DetectorParameters_create()

    # Load calibration data
    mtx, dist = loadCalibration()

    while(cap.isOpened()):
        # Read a frame off video feed
        ret, frame = cap.read()

        frame, markerIDs, rvec, tvec = arucoAnalysis(arucoDict, frame, parameters, mtx, dist, textRecorder)

        # Display/record video frame
        if camera != 0:
            height , width , layers =  frame.shape
            new_h=int(height/scale)
            new_w=int(width/scale)
            resize = cv2.resize(frame, (new_w, new_h)) # <- resize for improved performance
            cv2.imshow('Tello',resize)
        else:
            cv2.imshow('frame',frame)
        vidRecorder.write(frame)

        # Halt stream if "q" typed
            # Match speed by setting waitkey parameter to fps
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    vidRecorder.release()
    textRecorder.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    visionProcessing()