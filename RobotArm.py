import Segment as sa
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import art3d
import numpy as np
import matplotlib.pyplot as plt
from itertools import product, combinations
from scipy.linalg import norm

#This function finds a perpendicular vector 
#To the vector inserted. This is used for making prismatic joint graphics
def perpendicularVector(h):
    #Check if the first and second matrix values are 0
    if norm(h[0]) == 0 and norm(h[1]) == 0:
        #if there all zero, that is a problem. Raise error
        if norm(h[2]) == 0:
            # v is Vector(0, 0, 0)
            raise ValueError('zero vector')

        # v is Vector(0, 0, v.z)
        #Otherwise just return a unit vector in a different direction from z axis
        return np.matrix([[0],[1],[0]])
    #If zero matriceses is not a problem, we can just use knowledge of perpendicular
    #vectors to return a perpendicular vector
    return np.matrix([[-float(h[1])], [float(h[0])], [0]])/norm(np.matrix([[-float(h[1])], [float(h[0])], [0]]))

#This form is heavily taken from Amy Teegarden's Stack Overflow answer
def calculateCylinder(coor,h):
    #axis and radius
    h = h/norm(h) #convert vector into unit vector
    R = 0.15 #radius
    mtx = .5*h #The cylinder is 1u in length
    #Start and end positions of cylinder
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

#The following makes a psuedo rectangular prism for prismatic joints 
def calculateRectangle(coor,h):
    w = 0.5 #width of rectangle
    t = 0.15 #height/2
    c1 = h*w #vector of rectangular prism and negative vector
    c2 = -h*w
    #The following calculates the 8 corners of the rectangular prism
    p0 = perpendicularVector(h)*t + c1 
    p2 = sa.rot(h,np.pi)*p0
    p1 = sa.rot(h,np.pi/2)*p0
    p3 = sa.rot(h,-np.pi/2)*p0
    #Move points relative to coordinate of joint
    p0 += coor
    p1 += coor
    p2 += coor
    p3 += coor
    #repeats for other side of rectangular prism
    p4 = perpendicularVector(h)*t + c2
    p6 = sa.rot(h,np.pi)*p4
    p5 = sa.rot(h,np.pi/2)*p4
    p7 = sa.rot(h,-np.pi/2)*p4
    p4 += coor
    p5 += coor
    p6 += coor
    p7 += coor    
    #returns array of 3D points.
    points = np.array([p0,p1,p2,p3,p4,p5,p6,p7])
    return points

class RobotArm:
    def __init__(self,segments,zeroConfig):
        if len(segments) != len(zeroConfig):
            raise ValueError('length of segment list and zero configuration list is not equal')
        self.numSegs = len(segments) #The number of segments in this robot
        self.focusedJoint = 0 #The joint in focus for manipulation
        self.segmentList = segments #The list of segments that make up the robot
        self.p0T = np.matrix([[0],[0],[0]]) #the position of the end effector of the robot
        self.r0T = sa.eye(3) #The orientation of the end effector of the robot
        self.Q =[] #The array of angles of the robot
        self.P = [] #The array containing the positions of each position
        self.numJoints = -1  #The number of joints tha can be manipulated
        
        #Dictionary that carries a key and matches it to the appropriate joint
        self.keyToJoint = {}
        
        #Add origin to the position list
        self.P.append(self.p0T)
        for i in range(0,len(segments)):#loop performs foward kinematics to find the p0T and r0T
            if segments[i].getSegmentType() == 1: #if segment is prismatic perform translational motion
                self.p0T = self.p0T + np.dot(self.r0T,zeroConfig[i]*segments[i].getLength())
            else:
                self.p0T = self.p0T + np.dot(self.r0T,segments[i].getLength()) 
                self.r0T = np.dot(self.r0T, sa.rot(segments[i].getUnitVector(),zeroConfig[i]))
            
            self.P.append(self.p0T) #add new position to positions list
            self.Q.append(zeroConfig[i]) #add joint angle to angles list
            #If the joint is not the first or last joint, add the joint types of it's adjacent 
            #joints to the segment object
            if i != 0 and i != len(segments) - 1:
                segments[i].adjacentJoints(segments[i-1].getSegmentType(),segments[i+1].getSegmentType())
            #Other wise, mark that the joint is the last or first joint with a -1    
            elif i == 0:
                segments[i].adjacentJoints(-1,segments[i+1].getSegmentType())
            elif i == len(segments) - 1:
                segments[i].adjacentJoints(segments[i-1].getSegmentType(),-1)
            
            if segments[i].getSegmentType() == 0 or segments[i].getSegmentType() == 1:
                self.numJoints += 1
                self.keyToJoint[self.numJoints] = i
                
                
            
        ####################### Draw Plot
        #Creates figure for program to put things on
        self.fig = plt.figure()
    
        
        
        self.drawArm()
        return
    
    
    
    def drawArm(self):
        #Tell the figure to make a 3d projection
        self.ax = self.fig.gca(projection='3d')    
        self.ax.set_aspect('equal')
        self.drawnItems = []
        #initialize event handler
        cid = self.fig.canvas.mpl_connect('key_press_event', self.keyPress)
        
        # draw cube to create equaivalently scaled axis
        r = [0, 6] #region the cube covers
        for s, e in combinations(np.array(list(product(r, r, r))), 2):
            if np.sum(np.abs(s-e)) == r[1]-r[0]:
                self.ax.plot3D(*zip(s, e), color="w",alpha=0)  
            
        #The following draws the robot arms.          
        for i in range(1,len(self.P)-1):
            #draw a line dependent on P and Q 
            if self.segmentList[i].getSegmentType() == 2 or self.segmentList[i].getSegmentType() == 1: #check that a segment is an arm
                p1 = self.P[i] #Set the starting and ending points of the joint
                p0 = self.P[i-1]
                #create a range that moves backwards until a none joint segment is found
                rge = range(0,i)
                rge.reverse()
                for j in rge:
                    p0 = self.P[j]
                    if self.segmentList[j].getSegmentType() == 2 or self.segmentList[j].getSegmentType() == 1: #report first non joint segment
                        break
                p1 = np.squeeze(np.asarray(p1))
                p0 = np.squeeze(np.asarray(p0))
                #draw the line
                line, = self.ax.plot(*zip(p0,p1), color="cyan", linewidth=5.0,zorder=1)
                
        #Add the last segment to the robot, assuming it is not a joint
        if self.segmentList[-1].getSegmentType() == 2:
            p1 = self.P[-1]
            p0 = self.P[-2]
            p1 = np.squeeze(np.asarray(p1))
            p0 = np.squeeze(np.asarray(p0))
            #draw the line
            line, = self.ax.plot(*zip(p0,p1), color="cyan", linewidth=5.0,zorder=1) 
            
                            
        #The following loop draws a dot at each joint connection    
        for i in range(0,self.numSegs + 1):
            if i == 0:
                continue
            p1 = self.P[i]
            p0 = self.P[i-1]
            dist = np.linalg.norm(p1-p0)
            #if the distance between two points is 0  than you can skip it 
            if dist == 0:
                continue  
            #convery xyz to proper list format
            x = [float(p1[0])]
            y = [float(p1[1])]
            z = [float(p1[2])]   
            #Plot a point representing the segment end
            point, = self.ax.plot(x,y,z, 'ko',zorder=2)
            self.drawnItems.append(point)
        
        #The following plots revolute joints and prismatic joints    
        for i in range(0,self.numSegs):
            if self.segmentList[i].getSegmentType() == 0: #check if joint is revolute
                #Get the coordinates, and unit vector
                coor = self.P[i]
                h = self.segmentList[i].getUnitVector()
                r = sa.eye(3)#form new rotation matrix for finding new unit vector
                for val in range(0,i): #Go up to the rotation of the current joint
                    if self.segmentList[val].getSegmentType() == 0: #u vector only changes, if seg is revolute
                        th = self.segmentList[val].getUnitVector() #get og unit vector
                        r = np.dot(r, sa.rot(th, self.Q[val])) #iterate the rotation angle
                h = np.dot(r,h) #calculate new unit vector
                X,Y,Z = calculateCylinder(coor,h)
                #draw cylinder
                surface = self.ax.plot_surface(X, Y, Z, color = 'b',edgecolor='b', zorder=3)
                
            if self.segmentList[i].getSegmentType() == 1: #check if joint is prismatic
                coor = self.P[i] #locate coordinate the joint is centered and the unit vector
                h = self.segmentList[i].getUnitVector()
                r = sa.eye(3)
                #similarly to revolute joint, find rotation of unit vector for joint
                for val in range(0,i):
                    if self.segmentList[val].getSegmentType() == 0:
                        th = self.segmentList[val].getUnitVector()
                        r = np.dot(r, sa.rot(th, self.Q[val]))
                h = np.dot(r,h)                
                points = calculateRectangle(coor,h)
                #get X,Y,Z positions of the cube corners
                X = points[:,0]
                Y = points[:,1]
                Z = points[:,2]
                # connect each point in the cube with lines
                for i in range(0,len(points)):
                    for j in range(i+1,len(points)):
                        p1 = np.squeeze(np.asarray(points[i]))
                        p0 = np.squeeze(np.asarray(points[j]))                        
                        line, = self.ax.plot(*zip(p1,p0), color="red")
                        self.drawnItems.append(line,)
            
        
        plt.show()
        return
    
    #The following function handles key press events
    def keyPress(self,event):
        #if p is pressed then increase the angle of that joint
        if event.key == 'p':
            print "p was selected, so increase q"
            #if joint is prismatic increase length
            if self.segmentList[self.focusedJoint].getSegmentType() == 1:
                self.Q[self.focusedJoint] += 0.1
                #impose the limit on extension
                if self.Q[self.focusedJoint] > 1:
                    self.Q[self.focusedJoint] = 1  
            #if joint is revolute
            elif self.segmentList[self.focusedJoint].getSegmentType() == 0:
                self.Q[self.focusedJoint] += np.pi/8 #increase by this much
            self.fig.clear()
            self.recalculate()
            self.drawArm()
                       
        #if l is pressed decreased the angle of that joint
        elif event.key == 'l':
            print "l was selected, so decrease q"
            #similar to incrementing the length but in negative direction.
            if self.segmentList[self.focusedJoint].getSegmentType() == 1:
                self.Q[self.focusedJoint] -= 0.1
                if self.Q[self.focusedJoint] < -1:
                    self.Q[self.focusedJoint]
            elif self.segmentList[self.focusedJoint].getSegmentType() == 0:
                self.Q[self.focusedJoint] -= np.pi/8
            self.fig.clear()
            self.recalculate()
            self.drawArm()              
        #if the key pressed is number, change the focused joint to this number
        #This is the joint that responds to p and l
        elif event.key.isdigit():
            #check that the selected joint is valid. If it isn't reset the joint to 0
            if self.numJoints < int(event.key):
                self.focusedJoint = 0
            else:
                self.focusedJoint = self.keyToJoint[int(event.key)]
            print "self.focusedJoint is now: " + str(self.focusedJoint)                          
        return
    
    def clearDrawings(self):
        for item in self.drawnItems:
            item.remove()
            
    def recalculate(self):
        self.p0T = np.matrix([[0],[0],[0]])
        #The orientation of the end effector of the robot
        self.r0T = sa.eye(3)
        #The array containing the positions of each position
        self.P = []
        #Add origin to the position list
        self.P.append(self.p0T)
        for i in range(0,len(self.segmentList)):#loop performs foward kinematics to find the p0T and r0T
            if self.segmentList[i].getSegmentType() == 1:
                self.p0T = self.p0T + np.dot(self.r0T,self.Q[i]*self.segmentList[i].getLength())
                self.r0T = np.dot(self.r0T, sa.eye(3))
            else:
                self.p0T = self.p0T + np.dot(self.r0T,self.segmentList[i].getLength())
                self.r0T = np.dot(self.r0T, sa.rot(self.segmentList[i].getUnitVector(),self.Q[i]))   
                
            self.P.append(self.p0T) #add new position to positions list
        
    
            
        
        
        
