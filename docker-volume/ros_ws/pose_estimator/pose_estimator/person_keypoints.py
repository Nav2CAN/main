from dataclasses import dataclass

@dataclass
class keypoint():
    ID: str
    xImage:float
    yImage=float
    x,y,z=float

    def __init__(self, ID: str, xImage: float, yImage: float):
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
        self.nose=keypoint(0,0,0)

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