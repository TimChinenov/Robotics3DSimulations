import numpy as np

#rotation matrix using euler-rodriguez formula
def rot(h,theta):
    R = eye(3) + np.sin(theta)*hat(h) + (1-np.cos(theta))*np.dot(hat(h),hat(h))
    return R

#skew-symmetric matrix of a 3x1
def hat(k):
    a = float(k[0])
    b = float(k[1])
    c = float(k[2])
    ssm = np.matrix([[0,-c,b],
                     [c,0,-a],
                     [-b,a,0]])
    return ssm

#identity matrix creator
def eye(val):
    #function creates val x val identity matrix
    ident = []
    for i in range(0,val):
        tempI = [0]*val
        tempI[i] = 1
        ident.append(tempI)
    identM = np.asarray(ident)
    return identM

#Segment should able to take a arm or a joint
class Segment:
    #xyz, unit vectors. For shortness
    ex = np.matrix([[1],[0],[0]])
    ey = np.matrix([[0],[1],[0]])
    ez = np.matrix([[0],[0],[1]])
    
    #length is the length of the segments if they are segs
    #h is the zero configuration unit vector
    #q is the angle or percentage the prismatic joint extended
    def __init__(self,segType,length,h,q):
        #will dictate revolution joints as 0, prismatic joints as 1,
        #and segments as 2. 
        #If the segment is a revolute joint then it's length should be 0
        self.segType = segType
        self.length = length*h
        self.h = h #unit vector
        self.q = q #angle or prismatic distance
        
        #If the segment is a revolute joint
        if segType == 2:
            self.length = np.dot(rot(self.getUnitVector(),self.q),self.length)
            
    def adjacentJoints(self,prev_joint,next_joint):
        #tuple holds the type of segment that joint attaches too.
        self.jointEnds = (prev_joint,next_joint)
        return
    
    def getLength(self):
        return self.length
    
    def getUnitVector(self):
        return self.h
    
    def getSegmentType(self):
        return self.segType
    
    
    
    
    
        
        
        
        
        