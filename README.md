# robot-arm-qrcode
A 5-DOF robot arm with inverse kinematics on Arduino UNO, receiving QR code-based target positions from ESP32-CAM over serial communication.
'''
This project implements a 5-DOF robot arm where the first joint is prismatic and the remaining four are revolute.
An Arduino UNO handles control and inverse kinematics calculations. An ESP32-CAM scans QR codes containing target coordinates and transmits them via serial (TX/RX) to the Arduino. 
The Arduino processes the coordinates, computes inverse kinematics, and actuates the robot to move to the specified position.
