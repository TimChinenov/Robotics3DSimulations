import Segment as sa
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import art3d
import numpy as np
import matplotlib.pyplot as plt
from itertools import product, combinations
from scipy.linalg import norm

def perpendicularVector(h):
    if norm(h[0]) == 0 and norm(h[1]) == 0:
        if iszero(h[2]):
            # v is Vector(0, 0, 0)
            raise ValueError('zero vector')

        # v is Vector(0, 0, v.z)
        return np.matrix([[0],[1],[0]])

    return np.matrix([[-h[1]], [h[0]], [0]])

def calculateCylinder(coor,h):
    #axis and radius
    R = 0.15
    mtx = .5*h
    p0 = coor + mtx
    p1 = coor - mtx
    p0 = np.array([float(p0[0]), float(p0[1]), float(p0[2])])
    p1 = np.array([float(p1[0]), float(p1[1]), float(p1[2])])
    
    #vector in direction of axis
    v = p1 - p0
    #find magnitude of vector
    mag = norm(v)
    #unit vector in direction of axis
    v = v / mag
    #make some vector not in the same direction as v
    not_v = np.array([1, 0, 0])
    
    if (abs(v) == abs(not_v)).all():
        not_v = np.array([0, 1, 0])
    
    #make vector perpendicular to v
    n1 = np.cross(v, not_v)
    #normalize n1
    n1 /= norm(n1)
    #make unit vector perpendicular to v and n1
    n2 = np.cross(v, n1)
    #surface ranges over t from 0 to length of axis and 0 to 2*pi
    t = np.linspace(0, mag, 100)
    theta = np.linspace(0, 2 * np.pi, 100)
    #use meshgrid to make 2d arrays
    t, theta = np.meshgrid(t, theta)
    #generate coordinates for surface
    return [p0[i] + v[i] * t + R * np.sin(theta) * n1[i] + R * np.cos(theta) * n2[i] for i in [0, 1, 2]]      

def calculateRectangle(coor,h):
    w = 0.5
    t = 0.15
    c1 = coor + h*w
    c2 = coor - h*w
    p0 = perpendicularVector(h)*t
    p1 = rot(h,np.pi)*p0
    p3 = rot(h,np.pi/2)*p0
    p4 = rot(h,-np.pi/2)*p0
    p5 = -p0
    p6 = -p1
    p7 = -p2
    p8 = -p3
    print p0
    print p1
    print p2
    print p3
    print p4
    print p5
    print p6
    print p7
    print p8
    return
class RobotArm:

    def __init__(self,segments,zeroConfig):
        #The number of segments in this robot
        self.numSegs = len(segments)
        #The joint in focus for manipulation
        self.focusedJoint = 0
        #The list of segments that make up the robot
        self.segmentList = segments
        #the position of the end effector of the robot
        self.p0T = np.matrix([[0],[0],[0]])
        #The orientation of the end effector of the robot
        self.r0T = sa.eye(3)
        #The array of angles of the robot
        self.Q =[]
        #The array containing the positions of each position
        self.P = []
        self.P.append(self.p0T)
        for i in range(0,len(segments)):
            self.p0T = self.p0T + np.dot(self.r0T,segments[i].getLength())
            self.r0T = np.dot(self.r0T, sa.rot(segments[i].getUnitVector(),zeroConfig[i]))
            self.P.append(self.p0T)
            self.Q.append(zeroConfig[i])
            
        ####################### Draw Plot
        self.drawArm()
        return
    
    
    
    def drawArm(self):
        #Creates figure for program to put things on
        self.fig = plt.figure()
    
        #Tell the figure to make a 3d projection
        self.ax = self.fig.gca(projection='3d')    
        self.ax.set_aspect('equal')
        self.drawnItems = []
        
        # draw cube
        r = [0, 6]
        for s, e in combinations(np.array(list(product(r, r, r))), 2):
            if np.sum(np.abs(s-e)) == r[1]-r[0]:
                self.ax.plot3D(*zip(s, e), color="w",alpha=0)  
            
        #The following draws the robot arms.          
        for i in range(1,len(self.P)-1):
            #draw a line dependent on P and Q 
            if self.segmentList[i].getSegmentType() == 2:
                p1 = self.P[i]
                rge = range(0,i)
                rge.reverse()
                p0 = self.P[i-1]
                for j in rge:
                    p0 = self.P[j]
                    if self.segmentList[j].getSegmentType() == 2:
                        break
                p1 = np.squeeze(np.asarray(p1))
                p0 = np.squeeze(np.asarray(p0))
                line = self.ax.plot(*zip(p0,p1), color="cyan", linewidth=5.0,zorder=1)
        if self.segmentList[-1].getSegmentType() == 2:
            p1 = self.P[-1]
            p0 = self.P[-2]
            p1 = np.squeeze(np.asarray(p1))
            p0 = np.squeeze(np.asarray(p0))
            line = self.ax.plot(*zip(p0,p1), color="cyan", linewidth=5.0,zorder=1)            
            
                            
        #The following loop draws a dot at each joint connection    
        for i in range(0,self.numSegs + 1):
            if i == 0:
                continue
            p1 = self.P[i]
            p0 = self.P[i-1]
            dist = np.linalg.norm(p1-p0)
            if dist == 0 or dist == 1:
                continue  
            x = [float(p1[0])]
            y = [float(p1[1])]
            z = [float(p1[2])]   
            #Plot a point representing the segment end
            self.ax.plot(x,y,z, 'ko',zorder=2)
        
        #The following plots revolute joints and prismatic joints    
        for i in range(0,self.numSegs):
            if self.segmentList[i].getSegmentType() == 0:
                coor = self.P[i]
                h = self.segmentList[i].getUnitVector()
                X,Y,Z = calculateCylinder(coor,h)
                self.ax.plot_surface(X, Y, Z, color = 'b',edgecolor='b', zorder=3)
            if self.segmentList[i].getSegmentType() == 1:
                coor = self.P[i]
                h = self.segmentList[i].getUnitVector()
                calculateRectangle(coor,h)           
            
            
        plt.show()
        
    
            
        
        
        
