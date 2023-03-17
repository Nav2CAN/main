from dataclasses import dataclass
import numpy as np


@dataclass
class keypoint():
    ID: str
    xImage:float
    yImage=float
    x,y,z=float

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
        
        self.keypoints = np.array([])
        for kp in keypoints:
            self.keypoints = np.append(keypoints, keypoint(kp.ID, kp.x, kp.y))
        

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