import Segment as sg
import RobotArm as ra
import numpy as np
#################################
#Current Bugs:
#Problems when trying to make ex
#The cylinder should overlap the lines not the other way around
#################################

ex = np.matrix([[1],[0],[0]])
ey = np.matrix([[0],[1],[0]])
ez = np.matrix([[0],[0],[1]])

s1 = sg.Segment(2,2,ez,0)
s6 = sg.Segment(1,0,ex,0)
s7 = sg.Segment(1,0,ez,0)
s8 = sg.Segment(1,0,ey,0)
s2 = sg.Segment(2,4,ex,0)
s4 = sg.Segment(2,2,ey,0)
s5 = sg.Segment(2,2,ex,0)

segments = [s1,s6,s7,s8,s2,s4,s5]
Q = [np.pi/4,0,0,0,0,0,0]

r1 = ra.RobotArm(segments,Q)