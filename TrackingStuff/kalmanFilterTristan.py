from filterpy.kalman import KalmanFilter
import numpy as np
from scipy.linalg import block_diag
from filterpy.common import Q_discrete_white_noise
import numpy as np


def wrapAngle(angle):
    angle = angle%(2*np.pi)

    if angle>np.pi:
        angle-=2*np.pi
    elif angle<-np.pi:
        angle+=2*np.pi
    return angle

def unwrapAngle(angle, oldangle, residual):
    if not np.isnan(oldangle):
        if angle - oldangle < -1.5*np.pi:
            residual += 1
        elif angle - oldangle > 1.5*np.pi:
            residual -= 1
    return angle + residual * 2 * np.pi,residual

class personTracker(KalmanFilter):
    def __init__(self,timestamp, init_pos,dt=0.1,dim_x=6, dim_z=3):
        
        # state= x y theta dotx doty dottheta
        # measurements = x,y,theta
        super().__init__(dim_x=6,dim_z=3)
        self.generateMatricies(dt)
        self.u = 0. #no control inputs
        self.H = np.array([ [1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])
        self.R = np.array([ [0.0001,     0,      0],
                            [0,         0.0001,   0],
                            [0,         0,      0.001]])
        self.x = np.array([[init_pos[0],init_pos[1],init_pos[2], 0.0, 0.0, 0.0]]).T
        self.P = np.eye(self.dim_x) * 1000.
        self.B = np.zeros(self.dim_x).T

        #state
        self.personX:float = init_pos[0]
        self.personY:float= init_pos[1]
        self.personTheta:float = init_pos[2]
        #measurement 
        self.measX:float = init_pos[0]
        self.measY:float = init_pos[1]
        self.measTheta:float = init_pos[2]
        #prior
        self.personX_prior:float = init_pos[0]
        self.personY_prior:float= init_pos[1]
        self.personTheta_prior:float = init_pos[2]
        self.timestamp=timestamp
        self.angle_residual=0

    def generateMatricies(self, dt):
        #so the kalman filter can adjust to the rate
        self.F = np.array([ [1,  0, 0,  dt,   0,  0],
                            [0,  1, 0,   0,  dt,  0],
                            [0,  0, 1,   0,   0, dt],
                            [0,  0, 0,   1,   0,  0],
                            [0,  0, 0,   0,   1,  0],
                            [0,  0, 0,   0,   0,  1]])
        q = Q_discrete_white_noise(dim=self.dim_z, dt=dt, var=0.05)
        self.Q = block_diag(q, q)

    def kfUpdate(self, timestamp):
        dt=abs(timestamp-self.timestamp)*1e-9#calculate actual frequency
        if(dt==0): dt=0.1
        self.generateMatricies(dt)
        angleUnwrapped,self.angle_residual=unwrapAngle(self.measTheta,self.personTheta,self.angle_residual)

        self.update([self.measX,self.measY,angleUnwrapped])
        self.personX = self.x_post[0]
        self.personY = self.x_post[1]
        self.personTheta = wrapAngle(self.x_post[2])
        self.timestamp=timestamp

    def kfPredict(self):
        self.predict()
        self.personX_prior = self.x_prior[0]
        self.personY_prior = self.x_prior[1]
        self.personTheta_prior = wrapAngle(self.x_prior[2])
        


