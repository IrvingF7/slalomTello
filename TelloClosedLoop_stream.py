# This script is part of our course on Tello drone programming
# https://learn.droneblocks.io/p/tello-drone-programming-with-python/

# Import the necessary modules
import socket
import threading
import time
from time import sleep
import sys
import numpy as np
from queue import Queue
from queue import LifoQueue
import os
import cv2 as cv


State_data_file_name = 'statedata.txt'
index = 0
reference = 0.0     # Reference signal
control_LR = 0      # Control input for left/right
control_FB = 0      # Cotnrol input for forward/back
control_UD = 0      # Control input for up/down
control_YA = 0      # Control input for yaw
INTERVAL = 0.05  # update rate for state information
start_time = time.time()
dataQ = Queue()
stateQ = LifoQueue() # have top element available for reading present state by control loop

tvec_GLOBAL = [1.0,1.0,1.0]
rvec_GLOBAL = [1.0,1.0,1.0]

# IP and port of Tello for commands
tello_address = ('192.168.10.1', 8889)
# IP and port of local computer
local_address = ('', 8889)
# Create a UDP connection that we'll send the command to
CmdSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
CmdSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# Bind to the local address and port
CmdSock.bind(local_address)

###################
# socket for state information
local_port = 8890
StateSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # socket for sending cmd
StateSock.bind(('', local_port))
CmdSock.sendto('command'.encode('utf-8'), tello_address)   # command port on Tello

def writeFileHeader(dataFileName):
    fileout = open(dataFileName,'w')
    #write out parameters in format which can be imported to Excel
    today = time.localtime()
    date = str(today.tm_year)+'/'+str(today.tm_mon)+'/'+str(today.tm_mday)+'  '
    date = date + str(today.tm_hour) +':' + str(today.tm_min)+':'+str(today.tm_sec)
    fileout.write('"Data file recorded ' + date + '"\n')
    # header information
    fileout.write('  index,   time,    ref,ctrl_LR,ctrl_FB,ctrl_UD,ctrl_YA,  pitch,   roll,    yaw,    vgx,    vgy,    vgz,   templ,   temph,    tof,      h,    bat,   baro,   time,    agx,    agy,    agz\n\r')
    fileout.close()


def writeDataFile(dataFileName):
    fileout = open(State_data_file_name, 'a')  # append
    print('writing data to file')
    while not dataQ.empty():
        telemdata = dataQ.get()
        np.savetxt(fileout , [telemdata], fmt='%7.3f', delimiter = ',')  # need to make telemdata a list
    fileout.close()


def report_tag(str,index):
    telemdata=[]
    telemdata.append(index)
    telemdata.append(time.time()-start_time)
    telemdata.append(reference)
    telemdata.append(control_LR)
    telemdata.append(control_FB)
    telemdata.append(control_UD)
    telemdata.append(control_YA)
    data = str.split(';')
    data.pop() # get rid of last element, which is \\r\\n
    for value in data:
        temp = value.split(':')
        if temp[0] == 'mpry': # roll/pitch/yaw
            temp1 = temp[1].split(',')
            telemdata.append(float(temp1[0]))     # roll
            telemdata.append(float(temp1[1]))     # pitch
            telemdata.append(float(temp1[2]))     # yaw
            continue
        quantity = float(value.split(':')[1])
        telemdata.append(quantity)
    telemdata.append(tvec_GLOBAL[0])
    telemdata.append(tvec_GLOBAL[1])
    telemdata.append(tvec_GLOBAL[2])
    telemdata.append(rvec_GLOBAL[0])
    telemdata.append(rvec_GLOBAL[1])
    telemdata.append(rvec_GLOBAL[2])
    dataQ.put(telemdata)
    stateQ.put(telemdata)
    if (index %100) == 0:
        print(index, end=',')

# Send the message to Tello and allow for a delay in seconds
def send(message):
  # Try to send the message otherwise print the exception
  try:
    CmdSock.sendto(message.encode(), tello_address)
    # print("Sending message: " + message)
  except Exception as e:
    print("Error sending: " + str(e))

# receive state message from Tello
def rcvstate():
    print('Started rcvstate thread')
    index = 0
    while not stateStop.is_set():

        response, ip = StateSock.recvfrom(1024)
        if response == 'ok':
            continue
        report_tag(str(response),index)
        sleep(INTERVAL)
        index +=1
    print('finished rcvstate thread')


def camera():
    global rvec_GLOBAL
    global tvec_GLOBAL

    print('Started camera thread')
    path = os.path.abspath('..')
    fname = path + "\\slalomTello\\res\\calibration_parameters.txt"
    print(fname)
    #cap = cv.VideoCapture(0)
    cap = cv.VideoCapture("udp://@0.0.0.0:11111")
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

    print('beginning camera thread loop')
    #scale = 3
    ret = False

    while(True):
        ret, frame = cap.read()
        if(ret):
            #height , width , layers =  frame.shape
            #new_h=int(height/scale)
            #new_w=int(width/scale)
            #resize = cv.resize(frame, (new_w, new_h)) # <- resize for improved performance
            # Display the resulting frame

            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            font = cv.FONT_HERSHEY_SIMPLEX
            if np.all(ids != None):
                rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)

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
                pass

            tvec_GLOBAL = tvec[0][0]
            rvec_GLOBAL = rvec[0][0]
            cv.imshow('frame',frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

    print('finished stream thread')
    cap.release()
    cv.destroyAllWindows()
    print('Ended stream thread')

# Receive the message from Tello
def receive():
  # Continuously loop and listen for incoming messages
  while True:
    # Try to receive the message otherwise print the exception
    try:
      response, ip_address = CmdSock.recvfrom(128)
      print("Received message: " + response.decode(encoding='utf-8'))
    except Exception as e:
      # If there's an error close the socket and break out of the loop
      CmdSock.close()
      print("Error receiving: " + str(e))
      break



send('command')
send('streamon')
# Create and start a listening thread that runs in the background
# This utilizes our receive function and will continuously monitor for incoming messages
receiveThread = threading.Thread(target=receive)
receiveThread.daemon = True
receiveThread.start()

writeFileHeader(State_data_file_name)  # make sure file is created first so don't delay
stateThread = threading.Thread(target=rcvstate)
stateThread.daemon = False  # want clean file close
stateStop = threading.Event()
stateStop.clear()
stateThread.start()

stateThreadStream = threading.Thread(target=camera)
stateThreadStream.daemon = True  # want clean file close
stateThreadStream.start()



print('Type in a Tello SDK command and press the enter key. Enter "quit" to exit this program.')

# Loop infinitely waiting for commands or until the user types quit or ctrl-c
while True:

  try:
    # Read keybord input from the user
    if (sys.version_info > (3, 0)):
      # Python 3 compatibility
      message = input('')
    else:
      # Python 2 compatibility
      message = raw_input('')

    # If user types quit then lets exit and close the socket
    if 'quit' in message:
      print("Program exited")
      stateStop.set()  # set stop variable
      stateThread.join()   # wait for termination of state thread before closing socket
      writeDataFile(State_data_file_name)
      CmdSock.close()  # sockete for commands
      StateSock.close()  # socket for state
      print("sockets and threads closed")

      break

    # Send the command to Tello
    send(message)
    sleep(10.0) # wait for takeoff and motors to spin up
    # height in centimeters
    print('takeoff done')

###################################################################################################
#################################DON'T TOUCH ANYTHING OUTSIDE THIS#################################
###################################################################################################

    # Controller Variables
    kp = 2.5 # <--------------------------------------------------------------------------------------- Fill this out
    ki = 1.0
    kd = 0.1

    # Control stores
    integratedError = 0.0
    errorDerivative = 0.0
    errorStore = 0.0

    # Useful Reference Signal Variables
    period = 20.0 # <----------------------------------------------------------------------------------- Fill this out
    amplitude = 2.8 # <-------------------------------------------------------------------------------- Fill this out

    # to prevent hickups
    lastTime = 0.0
    lastYaw = 0.0

    for i in range(0,500):

        # Get data (read sensors)
        presentState = stateQ.get(block=True, timeout=None)  # block if needed until new state is ready
        ptime = presentState[1]     # present time (don't over write time function)
        yaw = presentState[9]       # current yaw angle (don't overwrite)
        if lastTime > ptime:
            ptime = lastTime
            yaw = lastYaw

        # Compute Reference Signal (Triangle wave)
        reference = -((2*amplitude)/np.pi) * np.arcsin(np.sin((2*np.pi*ptime)/(period)))
        reference = np.rad2deg(reference)

        #PID control
        error = reference - yaw
        if i>100:
            integratedError = integratedError + INTERVAL*error
            errorDerivative = (error - errorStore) / INTERVAL
            errorStore = error
        control_YA = kp*error + ki*integratedError + kd*errorDerivative # + k3*integrated2Error

        # Compute Error and Control Input
        #error = reference-yaw # <-------------------------------------------------------------------------------- Fill this out
        #control_YA = kp*error #<--------------------------------------------------------------------------- Fill this out

        lastTime = ptime
        lastYaw = yaw

###################################################################################################
#################################DON'T TOUCH ANYTHING OUTSIDE THIS#################################
###################################################################################################


        # Send Control to quad
        control_LR = int(np.clip(control_LR,-100,100))
        control_FB = int(np.clip(control_FB,-100,100))
        control_UD = int(np.clip(control_UD,-100,100))
        control_YA = int(np.clip(control_YA,-100,100))
        message = 'rc '+str(control_LR)+' '+str(control_FB)+' '+str(control_UD)+' '+str(control_YA)
        send(message)

        # Wait so make sample time steady
        sleep(0.1)

    message='rc 0 0 0 0' # stop motion
    control_input = 0
    send(message)
    sleep(1.5)
    message ='land'
    send(message)
    print('landing')

    # Handle ctrl-c case to quit and close the socket
  except KeyboardInterrupt as e:
    message='emergency' # try to turn off motors
    send(message)
    stateStop.set()  # set stop variable
    stateThread.join()   # wait for termination of state thread
    writeDataFile(State_data_file_name)
    CmdSock.close()
    StateSock.close()  # socket for state
    print("sockets and threads closed")
    break
