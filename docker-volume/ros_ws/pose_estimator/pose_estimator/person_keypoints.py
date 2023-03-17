
import numpy as np


class keypoint():
    ID: str
    xImage:float
    yImage=float
    x =float
    y=float
    z=float

    def __init__(self, ID: int, xImage: float, yImage: float):

        
        # np.where(self.IDnames=="neck")[0][0]
        self.ID=ID
        self.xImage=xImage
        self.yImage=yImage
        self.x=None
        self.y=None
        self.z=None
    
    def calculate3DPoint(self,distance):
        #TODO implement this
        
        self.x=0
        self.y=0
        self.z=0

        return self.x,self.y,self.z

class person_keypoint:
    '''
    Keypoints for one detection are passed to this object for calculation of 3D location and orientation
    '''
    def __init__(self, keypoints):
        # self.IDnames = np.array(["nose", "left_eye", "right_eye", "left_ear",
        #                    "right_ear", "left_shoulder", "right_shoulder",
        #                    "left_elbow", "right_elbow", "left_wrist",
        #                    "right_wrist", "left_hip", "right_hip", "left_knee",
        #                    "right_knee", "left_ankle", "right_ankle", "neck"])
        
        self.keypoints = []
        for kp in keypoints:
            self.keypoints.append(keypoint(kp.ID, kp.x, kp.y))
    
    def calculateOrientation(self):
        '''Brief: do the check which keypoints are present, then calculate orientation'''
        left_shoulder =  next((point for point in self.keypoints if point.ID == 5), None)
        right_shoulder = next((point for point in self.keypoints if point.ID == 6), None)
        left_hip =       next((point for point in self.keypoints if point.ID == 11), None)
        right_hip =      next((point for point in self.keypoints if point.ID == 12), None)

        #TODO continue this on monday
        print(left_shoulder)
        print(right_shoulder)
        print(left_hip)
        print(right_hip)

class person_tracking:
    def __init__(self, x, y, theta):
        

        self.x = x
        self.y = y
        self.theta = theta
        self.xdot=None
        self.ydot=None
        self.thetadot=None

    def calculate3DPose(self):
        None
    def calculateOrientation(self):
        None
    def calculatePosition(self):
        None