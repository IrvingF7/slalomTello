# slalomTello
### Introduction
slalomTello is a class project done by [Braeden Benedict](https://github.com/braedenbenedict), [Kathleen Chang](https://github.com/kathleenchang), and [Irving Fang](https://github.com/IrvingF7) for a Berkeley EE class.
The project involves writing controller to let [Tello](https://www.ryzerobotics.com/tello), a quadcopter drone developed by Ryze Robotics, navigate a set of slalom poles as soon as possible.

The control is PID-based and Aruco tags are used for position measurements.

### Showcase
1. Drone Perspective (Shot by onboard camera)

![](https://github.com/IrvingF7/slalomTello/blob/master/gif/drone%20perspective.gif)

2. Viewer Perspective

![](https://github.com/IrvingF7/slalomTello/blob/master/gif/viewer%20perspective.gif)

3. Video

  Please refer to this [link](https://www.youtube.com/watch?v=OK7-oIf_UHQ&feature=youtu.be&ab_channel=KathleenChang)

### Details
1. For details involved in this project, please refer to this [report](https://github.com/IrvingF7/slalomTello/blob/master/report/final%20report.pdf), which has very detailed explanation of almost every aspect of this project.

2. For replicating this project, please clone this repo, connect your computer to a Tello via WiFi, and run `TelloClosedLoop_stream.py`

### Miscellaneous
1. If you are having trouble importing `cv2.aruco`, please refer to this [link](https://stackoverflow.com/questions/45972357/python-opencv-aruco-no-module-named-cv2-aruco)
