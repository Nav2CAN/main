import munkres
import math
import numpy as np


class KalmanFilter(object):
    def __init__(
            self,
            x,
            y,
            theta,
            timestamp,
            dt=0.1,
            u_x=0,
            u_y=0,
            u_theta=0,
            std_acc=0.0001,
            x_std_meas=0.000001,
            y_std_meas=0.000001,
            theta_std_meas=0.000001,
            debug=False):
        """
        :param x: initial x measurement
        :param y: initial y measurement
        :param theta: initial orientation measurement
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
        self.personXdot = 0
        self.personYdot = 0
        self.personThetadot = 0
        self.timestamp = timestamp

        self.measX = x
        self.measY = y
        self.measTheta = theta
        self.measTimestamp = timestamp

        # Define sampling time
        self.dt = dt
        self.std_acc = std_acc
        # Define the  control input variables
        self.u = np.matrix([[u_x], [u_y], [u_theta]])

        # Intial State
        self.x = np.matrix([[self.personX], [self.personY], [
                           self.personTheta], [self.personXdot], [self.personYdot], [self.personThetadot]])

        # Define the State Transition Matrix A
        self.A, self.Q = None, None

        self.generateMatricies(dt)

        # Define the Control Input Matrix B
        self.B = np.matrix([[(self.dt**2) / 2, 0, 0],
                            [0, (self.dt**2) / 2, 0],
                            [0, 0, (self.dt**2) / 2],
                            [self.dt, 0, 0],
                            [0, self.dt, 0],
                            [0, 0, self.dt]])

        # Define Measurement Mapping Matrix
        self.H = np.matrix([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])

        # Initial Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas**2, 0, 0],
                           [0, y_std_meas**2, 0],
                           [0, 0, theta_std_meas**2]])

        # Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1])

    def angleWrap(self, old_angle, new_angle):
        # function for wrapping angle around if input angle crosses boundary

        if new_angle - old_angle < -math.pi:
            r = 1
        elif new_angle - old_angle > math.pi:
            r = -1

        out_angle = new_angle + r * 2 * math.pi

        return out_angle

    def predict(self):
        # Update time state
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # Calculate error covariance
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        # Update position
        self.personX = self.x[0]
        self.personY = self.x[1]
        self.personTheta = self.x[2]
        self.personXdot = self.x[3]
        self.personYdot = self.x[4]
        self.personThetadot = self.x[5]

    def generateMatricies(self, dt):

        self.A = np.matrix([[1, 0, 0, self.dt, 0, 0],
                            [0, 1, 0, 0, self.dt, 0],
                            [0, 0, 1, 0, 0, self.dt],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1]])

        self.Q = np.matrix([[(self.dt**4) / 4, 0, 0, (self.dt**3) / 2, 0, 0],
                            [0, (self.dt**4) / 4, 0, 0, (self.dt**3) / 2, 0],
                            [0, 0, (self.dt**4) / 4, 0, 0, (self.dt**3) / 2],
                            [(self.dt**3) / 2, 0, 0, self.dt**2, 0, 0],
                            [0, (self.dt**3) / 2, 0, 0, self.dt**2, 0],
                            [0, 0, (self.dt**3) / 2, 0, 0, self.dt**2]]) * self.std_acc**2

    def update(self):

        # print(f"before unwrap: {self.personTheta}, {self.x[2]}")
        self.personTheta = self.angleWrap(self.x[2], self.personTheta)
        # print(f"after unwrap: {self.personTheta}, {self.x[2]}")

        z = [[self.measX], [self.measY], [self.measTheta]]

        # time difference and map from [ns] to [s]
        dt = abs(self.meastimestamp-self.timestamp)*1e-9

        self.generateMatricies(dt)

        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Calculate the Kalman Gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # print(f"Kalman gain:\n {K}")

        # self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))
        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))

        I = np.eye(self.H.shape[1])

        # Update error covariance matrix
        self.P = (I - (K * self.H)) * self.P

        # wrap angle between -pi and pi
        self.x[2] = np.mod(self.x[2] + np.pi, 2 * np.pi) - np.pi

        # Update position
        self.personX = self.x[0]
        self.personY = self.x[1]
        self.personTheta = self.x[2]
        self.personXdot = self.x[3]
        self.personYdot = self.x[4]
        self.personThetadot = self.x[5]
        self.timestamp = self.meastimestamp


class MunkresAssignment(object):
    def __init__(self,
                 detection_dist=5,
                 gain=0.5,
                 debug=False):
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

        # used to disable assigning certain distances to tracklets
        self.NotAllowedForMunkres = munkres.DISALLOWED
        self.detection_dist = detection_dist
        self.gain = gain
        self.debug = debug

    def MunkresDistances(self, detections, tracks):
        # Calculate distances between objects and detections and save shortest
        # distances
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

                dist = abs(newPosX - currentPosX) + abs(newPosY - currentPosY) + \
                    self.gain * abs(newPosTheta -
                                    currentPosTheta) / (2 * math.pi)
                if dist <= self.detection_dist:
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
        if len(detections) == len(tracklets):
            if self.debug:
                print("same length")
            for index in indexes:
                tracklets[index[0]].measX = detections[index[1]].x
                tracklets[index[0]].measY = detections[index[1]].y
                tracklets[index[0]].measTheta = detections[index[1]].orientation
                tracklets[index[0]].measTimestamp = timestamp
                updates.append(index[0])

        elif len(tracklets) < len(detections):
            if self.debug:
                print("More detections")
            for index in indexes:
                tracklets[index[0]].measX = detections[index[1]].x
                tracklets[index[0]].measY = detections[index[1]].y
                tracklets[index[0]].measTheta = detections[index[1]].orientation
                tracklets[index[0]].measTimestamp = timestamp
                detections.pop(index[1])
                updates.append(index[0])

            for detection in detections:
                tracklets.append(
                    KalmanFilter(
                        detection.x,
                        detection.y,
                        detection.orientation,
                        timestamp))

        elif len(tracklets) > len(detections):
            if self.debug:
                print("More tracklets")
            for index in indexes:
                tracklets[index[0]].measX = detections[index[1]].x
                tracklets[index[0]].measY = detections[index[1]].y
                tracklets[index[0]].measTheta = detections[index[1]].orientation
                tracklets[index[0]].measTimestamp = timestamp
                updates.append(index[0])

        return updates


class PeopleTracker(object):
    def __init__(self, debug=False, keeptime=5):
        # initialise the tracker with an empty list of people

        self.assignment = MunkresAssignment(debug=debug)
        self.keeptime = keeptime
        self.personList = []

    def predict(self):
        # perform prediction with the Kalman filter
        for person in self.personList:
            person.predict()

    def update(self, detections, timestamp):
        # update the tracklets with new detections
        if len(self.personList):
            # remove person if it hasn't been detected in too long
            for person in self.personlist:
                if abs(timestamp-person.timestamp)*1e-9 > self.keeptime:
                    self.personlist.pop(person)

            updates = self.assignment.MunkresTrack(
                detections, self.personList, timestamp)

            for update in updates:
                self.personList[update].update(timestamp)

        else:
            for detection in detections:
                self.personList.append(
                    KalmanFilter(
                        detection.x,
                        detection.y,
                        detection.orientation,
                        timestamp))
