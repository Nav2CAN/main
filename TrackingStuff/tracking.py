import munkres
import math
import numpy as np


class KalmanFilter(object):
    def __init__(self, x, y, theta, timestamp,
                 dt = 0.033, u_x = 1, u_y  = 1, u_theta = 1,
                 std_acc = 0.1, x_std_meas = 0.1, y_std_meas = 0.1, theta_std_meas = 0.1, debug = False):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u_x: acceleration in x-direction
        :param u_y: acceleration in y-direction
        :param u_theta: acceleration in orientation
        :param std_acc: process noise magnitude
        :param x_std_meas: standard deviation of the measurement in x-direction
        :param y_std_meas: standard deviation of the measurement in y-direction
        :param theta_std_meas: standard deviation of the measurement in orientation (theta)
        """

        self.debug = debug

        # Define variables for sotring current position
        self.personX = x
        self.personY = y
        self.personTheta = theta
        self.timestamp = timestamp
        
        # Define sampling time
        self.dt = dt

        # Define the  control input variables
        self.u = np.matrix([[u_x], [u_y], [u_theta]])

        # Intial State
        self.x = np.matrix([[self.personX], [self.personY], [self.personTheta], [0], [0], [0]])

        # Define the State Transition Matrix A
        self.A = np.matrix([[1, 0, 0, self.dt, 0, 0],
                            [0, 1, 0, 0, self.dt, 0],
                            [0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])

        # Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2)/2, 0, 0],
                            [0, (self.dt**2)/2, 0],
                            [0, 0, (self.dt**2)/2],
                            [self.dt, 0, 0],
                            [0, 0, self.dt],
                            [0, self.dt, 0]])

        # Define Measurement Mapping Matrix
        self.H = np.matrix([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])

        #Initial Process Noise Covariance
        self.Q = np.matrix([[(self.dt**4)/4, 0, 0, (self.dt**3)/2, 0, 0],
                            [0, (self.dt**4)/4, 0, 0, (self.dt**3)/2, 0],
                            [0, 0, (self.dt**4)/4, 0, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, 0, self.dt**2, 0, 0],
                            [0, (self.dt**3)/2, 0, 0, self.dt**2, 0],
                            [0, 0, (self.dt**3)/2, 0, 0, self.dt**2]]) * std_acc**2

        #Initial Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas**2, 0, 0],
                           [0, y_std_meas**2, 0],
                           [0, 0, theta_std_meas**2]])

        #Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1])

    def predict(self):
        # Update time state
        self.x =  np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # Calculate error covariance
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        # Update position
        self.personX = self.x[0]
        self.personY = self.x[1]
        self.personTheta = self.x[2]

    
    def update(self, timestamp):

        z = [[self.personX], [self.personY], [self.personTheta]]

        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Calculate the Kalman Gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))
        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))

        I = np.eye(self.H.shape[1])

        # Update error covariance matrix
        self.P = (I - (K * self.H)) * self.P
        
        # Update position
        self.personX = self.x[0]
        self.personY = self.x[1]
        self.personTheta = self.x[2]
        self.timestamp = timestamp
    

class MunkresAssignment(object):
    def __init__(self,
                detection_dist = 5,
                gain = 1,
                debug = False):
        """
        :param centers: list of detected x, y and theta 
        :param current_tracks: list of dictionaries of current tracked people
            - x: map location of person, x value
            - y: map location of person, y value
            - theta: orientation of person
            - timestamp: last seen (used for discarding old tracklets)
        :param timestamp: current timestamp from detection (used for discarding old tracklets)
        :param RadiusOfDetection: used for disallowing assigning detections too far away
        :param gain: used for changing the weight of the orientation in distance function
        """

        self.NotAllowedForMunkres = munkres.DISALLOWED # used to disable assigning certain distances to tracklets
        self.detection_dist = detection_dist
        self.gain = gain 
        self.debug = debug

    def MunkresDistances(self, detections, tracks):
        # Calculate distances between objects and detections and save shortest distances
            distances = []
            for person in tracks:
                currentPosX = person.personX
                currentPosY = person.personY
                currentPosTheta = person.personTheta
                dists = []
                for detection in detections:
                    newPosX = detection.x
                    newPosY = detection.y
                    newPosTheta = detection.orientation
                    
                    dist = abs(newPosX-currentPosX) + abs(newPosY-currentPosY) + self.gain*abs(newPosTheta-currentPosTheta)/(2*math.pi)
                    if dist<=self.detection_dist:
                        dists.append(dist)
                    else:
                        dists.append(self.NotAllowedForMunkres)
                distances.append(dists)
            
            # Find shortest distances
            m = munkres.Munkres()
            self.indexes = m.compute(distances)

            return self.indexes

    def MunkresTrack(self, detections, tracklets, timestamp):
        indexes = self.MunkresDistances(detections, tracklets)
        updates = []
        if len(detections)==len(tracklets):
            if self.debug:
                print("same length")      
            for i, index in enumerate(indexes):
                tracklets[index[0]].personX = detections[index[1]].x
                tracklets[index[0]].personY = detections[index[1]].y
                tracklets[index[0]].personTheta = detections[index[1]].orientation
                tracklets[index[0]].timestamp = timestamp
                updates.append(index[0])
            return updates
            

        elif len(tracklets)<len(detections):
            if self.debug:
                print("More detections")
            for i, index in enumerate(self.indexes):
                tracklets[index[0]].personX = detections[index[1]].x
                tracklets[index[0]].personY = detections[index[1]].y
                tracklets[index[0]].personTheta = detections[index[1]].orientation
                tracklets[index[0]].timestamp = timestamp
                detections.pop(index[1])
                updates.append(index[0])
            

            for detection in detections:
                tracklets.append(KalmanFilter(detection.x, detection.y, detection.orientation, timestamp))
            return updates

        elif len(tracklets)>len(detections):
            if self.debug:
                print("More tracklets")
            for i, index in enumerate(self.indexes):
                tracklets[index[0]].personX = detections[index[1]].x
                tracklets[index[0]].personY = detections[index[1]].y
                tracklets[index[0]].personTheta = detections[index[1]].orientation
                tracklets[index[0]].timestamp = timestamp
                updates.append(index[0])
            return updates


class PeopleTracker(object):
    def __init__(self, debug = False):
        # initialise the tracker with an empty list of people

        self.assignment = MunkresAssignment(debug = debug)
        self.personList = []
        
    def predict(self):
        # perform prediction with the Kalman filter
        for person in self.personList:
            person.predict()

    def update(self, detections, timestamp):
        # update the tracklets with new detections
        if len(self.personList):
            updates = self.assignment.MunkresTrack(detections, self.personList, timestamp)
            
            for update in updates:
                self.personList[update].update(timestamp)
        
        else:
            for detection in detections:
                self.personList.append(KalmanFilter(detection.x, detection.y, detection.orientation, timestamp))


        # TODO add a popping function of "old" people
        
        