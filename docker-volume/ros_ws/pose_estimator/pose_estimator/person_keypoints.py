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


class person_keypoints:
    def __init__(self):
        self.IDnames = np.array(["nose", "left_eye", "right_eye", "left_ear",
                            "right_ear", "left_shoulder", "right_shoulder",
                            "left_elbow", "right_elbow", "left_wrist",
                            "right_wrist", "left_hip", "right_hip", "left_knee",
                            "right_knee", "left_ankle", "right_ankle", "neck"])
        
        self.nose=keypoint(0,None,None)
        self.left_eye = keypoint(1, None, None)
        self.right_eye = keypoint(2, None, None)
        self.left_ear = keypoint(3, None, None)
        self.right_ear = keypoint(4, None, None)
        self.left_shoulder = keypoint(5, None, None)
        self.right_shoulder = keypoint(6, None, None)
        self.left_elbow = keypoint(7, None, None)
        self.right_elbow = keypoint(8, None, None)
        self.left_wrist = keypoint(9, None, None)
        self.right_wrist = keypoint(10, None, None)
        self.left_hip = keypoint(11, None, None)
        self.right_hip = keypoint(12, None, None)
        self.left_knee = keypoint(13, None, None)
        self.right_knee = keypoint(14, None, None)
        self.left_ankle = keypoint(15, None, None)
        self.right_ankle = keypoint(16, None, None)
        self.neck = keypoint(17, None, None)

        self.x=None
        self.y=None
        self.theta=None
        self.xdot=None
        self.ydot=None
        self.thetadot=None

    def calculate3DPose(self):
        None
    def calculateOrientation(self):
        None
    def calculatePosition(self):
        None