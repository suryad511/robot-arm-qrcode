# Robot Arm QR Code Control

This project features a 5-DOF robot arm with one prismatic and four revolute joints. The arm is controlled by an Arduino UNO, which performs inverse kinematics calculations to move the joints. An ESP32-CAM is used to scan QR codes that contain target coordinates for the robot arm.

## Components

- Arduino UNO
- ESP32-CAM
- Servo motors
- PCA9685 Servo module
- Stepper motor
- A4988 Stepper module
- LCD 16x2 I2C
- QR code with positional data

## How It Works

1. The ESP32-CAM scans a QR code containing a target position.
2. It sends the coordinates over serial (TX/RX) to the Arduino UNO.
3. The Arduino parses the data and calculates inverse kinematics.
4. The Arduino commands the servos and actuator to move the arm to the target.

## Features

- 5 Degrees of Freedom (1 prismatic, 4 revolute)
- Inverse kinematics on microcontroller
- QR code-based position input
- Serial communication between ESP32-CAM and Arduino UNO

## Future Improvements

- Improve QR code recognition reliability
- Use better camera module
- Add a GUI for real-time monitoring
- Integrate object detection
