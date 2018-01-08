import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import art3d
import numpy as np
import matplotlib.pyplot as plt



class RobotArm:
    
    def __init__(self,xyz):
        #Creates figure for program to put things on
        self.fig = plt.figure()
        
        #Tell the figure to make a 3d projection
        self.ax = self.fig.gca(projection='3d')
        
        self.x = [xyz[0]]
        self.y = [xyz[1]]
        self.z = [xyz[2]]
        coor, = self.ax.plot(self.x, self.y, self.z, 'ro')
        self.points = []
        self.points.append(coor,)
    
        cid = self.fig.canvas.mpl_connect('key_press_event', self.onclick)
        plt.show()
    
    def onclick(self,event):
        if event.key == 'q':
            for point in self.points:
                self.z[0] = self.z[0] + 0.5
                self.ax.plot(self.x, self.y, self.z, 'ro')
        if event.key == 'a':
            for point in self.points:
                self.z[0] = self.z[0] - 0.5
                self.ax.plot(self.x, self.y, self.z, 'ro')            
        plt.show()
            
            
r1 = RobotArm([2,2,2])