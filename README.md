# Robotics3DSimulations

This program prototypes the construction and simulation of robotic arms. Robotic arms can be built from arm segments of chosen length, revolute joints, and prismatic joints. The purpose of this file is to demonstrate forward kinematic concepts on a platform that most people have access to. The long term goal of this project is to implement inverse kinematics for certain arms and optimize the graphical display. 

There are three main files of importance. "main.py" is where the script is run from. This file initializes the segments that make up the robot arm and the robot arm itself. "Segments.py" contains the class Segment. A segment is defined by a segment type, a length, a unit vector, and an initial angle. Segment objects make up a robot arm. The "RobotArm.py" file contains the class for the RobotArm. This class constructs the arm out of segments as well as draws, animates, and handles keyboard events.

Controls:

Each moveable joint is ordered starting from 0. Joints are selected by joint number and not by which segment they are. So the first joint that makes up robot arm can always be selected by clicking "0". To increase the joint's angle or prismatic translation, click "P". To decrease the value, click "L".
