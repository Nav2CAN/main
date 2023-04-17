# import munkres
import math
import numpy as np
from scipy.optimize import linear_sum_assignment


class KalmanFilter(object):
    def __init__(
            self,
            x,
            y,
            theta=0,
            withTheta=True,
            timestamp=0,
            dt=0.1,
            u_x=0,
            u_y=0,
            u_theta=0,
            std_acc=0.0001,
            std_theta_acc=0.0001,
            x_std_meas=0.000001,
            y_std_meas=0.000001,
            theta_std_meas=0.000001,
            decay=0.8,
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
        self.decay = decay
        self.initialized = False
        self.measX = x
        self.measY = y
        self.measTheta = theta
        self.measTimestamp = timestamp
        self.measWithTheta = withTheta
        self.residual = 0

        # Define sampling time
        self.dt = dt
        self.std_acc = std_acc
        # Define the  control input variables
        self.u = np.matrix([[u_x], [u_y], [u_theta]])

        # Intial State
        self.x = np.matrix([[self.personX], [self.personY], [self.personTheta], [
                           self.personXdot], [self.personYdot], [self.personThetadot]])

        # Define the State Transition Matrix A
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
        self.Q[2, 2] = ((self.dt**4)/4)*std_theta_acc**2
        self.Q[2, 5] = ((self.dt**3) / 2)*std_theta_acc**2
        self.Q[5, 2] = ((self.dt**3) / 2)*std_theta_acc**2
        self.Q[5, 5] = (self.dt**2)*std_theta_acc**2
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
        # Define Measurement Mapping Matrix without Theta
        self.Halternative = np.matrix([[1, 0, 0, 0, 0, 0],
                                       [0, 1, 0, 0, 0, 0]])

        # Initial Measurement Noise Covariance
        self.R = np.matrix([[x_std_meas**2, 0, 0],
                           [0, y_std_meas**2, 0],
                           [0, 0, theta_std_meas**2]])
        # Initial Measurement Noise Covariance
        self.Ralternative = np.matrix([[x_std_meas**2, 0],
                                       [0, y_std_meas**2]])

        # Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1])

        # matrix for decay the influence of prediction on movement over time when no detection
        self.DecayMatrix = np.matrix([[1, 0, 0, 0.0, 0, 0],
                                      [0, 1, 0, 0, 0.0, 0],
                                      [0, 0, 1, 0, 0, 0.0],
                                      [0, 0, 0, self.decay, 0, 0],
                                      [0, 0, 0, 0, self.decay, 0],
                                      [0, 0, 0, 0, 0, self.decay/10]])

    def angleUnWrap(self, old_angle, new_angle):
        # function for unwrapping angle around if input angle crosses boundary

        if new_angle - old_angle < -math.pi:
            self.residual += 1
        elif new_angle - old_angle > math.pi:
            self.residual -= 1

        out_angle = new_angle + self.residual * 2 * math.pi

        return out_angle

    def predict(self):
        # Update time state
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # TODO determine use of Decay velocity measurement
        self.x = np.dot(self.DecayMatrix, self.x)

        # Calculate error covariance
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        # Update position
        self.personX = self.x[0]
        self.personY = self.x[1]
        self.personTheta = self.x[2]
        self.personXdot = self.x[3]
        self.personYdot = self.x[4]
        self.personThetadot = self.x[5]

    def update(self):
        if self.measWithTheta:
            H = self.H
            R = self.R
            # self.measTheta = self.angleUnWrap(self.x[2], self.measTheta)
            print(
                np.unwrap(np.array([float(self.x[2]), float(self.measTheta)])))
            self.measTheta = np.unwrap(
                np.array([float(self.x[2]), float(self.measTheta)]))[1]
            z = [[self.measX], [self.measY], [self.measTheta]]
        else:
            H = self.Halternative
            R = self.Ralternative
            z = [[self.measX], [self.measY]]

        S = np.dot(H, np.dot(self.P, H.T)) + R

        # Calculate the Kalman Gain
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))
        # print(f"Kalman gain:\n {K}")

        # self.x = np.round(self.x + np.dot(K, (z - np.dot(H, self.x))))
        self.x = self.x + np.dot(K, (z - np.dot(H, self.x)))

        I = np.eye(H.shape[1])

        # Update error covariance matrix
        self.P = (I - (K * H)) * self.P

        # Update position
        self.personX = self.x[0]
        self.personY = self.x[1]
        self.personTheta = self.x[2]
        self.personXdot = self.x[3]
        self.personYdot = self.x[4]
        self.personThetadot = self.x[5]
        self.timestamp = self.measTimestamp


class MunkresAssignment(object):
    def __init__(self,
                 newTrack=10,
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
        self.debug = debug
        self.newTrack = newTrack

    def MunkresDistances(self, detections, tracks, timestamp):
        # Calculate distances between objects and detections and save shortest
        # distances while removing noise measurements
        distances = []
        for person in tracks:
            currentPosX = person.personX
            currentPosY = person.personY
            dists = []
            for detection in detections:
                newPosX = detection.x
                newPosY = detection.y
                dist = float(np.hypot((newPosX - currentPosX),
                             (newPosY - currentPosY)))
                print(f"dist{dist}")
                dists.append(dist)
            distances.append(dists)
        distMat = np.array(distances)
        print(f"shape of dist mat{np.shape(distMat)}")
        mins = distMat.min(axis=1)

        popCounter = 0
        if mins[mins > self.newTrack]:
            for i, min in enumerate(mins):
                if min > self.newTrack:
                    tracks.append(
                        KalmanFilter(
                            detections[i - popCounter].x,
                            detections[i - popCounter].y,
                            detections[i - popCounter].orientation,
                            withTheta=detections[i - popCounter].withTheta,
                            timestamp=timestamp))

                    detections[i - popCounter].pop()
                    distMat = np.delete(distMat, i-popCounter, 1)
                    popCounter += 1
        # Find shortest distance

        self.indexes = linear_sum_assignment(distMat)
        return self.indexes, detections

    def MunkresTrack(self, detections, tracklets, timestamp):
        updates = []
        if len(detections):
            indexes, detections = self.MunkresDistances(
                detections, tracklets, timestamp)
            if len(detections) == len(tracklets):
                if self.debug:
                    print("Same amount of detections and tracklets")
                for index in zip(indexes[0], indexes[1]):
                    tracklets[index[0]].measX = detections[index[1]].x
                    tracklets[index[0]].measY = detections[index[1]].y
                    tracklets[index[0]
                              ].measTheta = detections[index[1]].orientation
                    tracklets[index[0]].measTimestamp = timestamp
                    tracklets[index[0]
                              ].measWithTheta = detections[index[1]].withTheta

                    updates.append(index[0])

            elif len(tracklets) < len(detections):
                if self.debug:
                    print("More detections than tracklets")
                for index in zip(indexes[0], indexes[1]):
                    tracklets[index[0]].measX = detections[index[1]].x
                    tracklets[index[0]].measY = detections[index[1]].y
                    tracklets[index[0]
                              ].measTheta = detections[index[1]].orientation
                    tracklets[index[0]].measTimestamp = timestamp
                    tracklets[index[0]
                              ].measWithTheta = detections[index[1]].withTheta
                    detections.pop(index[1])
                    updates.append(index[0])

                for detection in detections:
                    tracklets.append(
                        KalmanFilter(
                            detection.x,
                            detection.y,
                            detection.orientation,
                            withTheta=detection.withTheta,
                            timestamp=timestamp))

            elif len(tracklets) > len(detections):
                if self.debug:
                    print("More tracklets than detections")
                for index in zip(indexes[0], indexes[1]):
                    tracklets[index[0]].measX = detections[index[1]].x
                    tracklets[index[0]].measY = detections[index[1]].y
                    tracklets[index[0]
                              ].measTheta = detections[index[1]].orientation
                    tracklets[index[0]].measTimestamp = timestamp
                    tracklets[index[0]
                              ].measWithTheta = detections[index[1]].withTheta
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
            n_popped = 0
            for i in range(len(self.personList)):
                if abs(timestamp-self.personList[i-n_popped].timestamp)*1e-9 > self.keeptime:
                    self.personList.pop(i-n_popped)
                    n_popped += 1

            updates = self.assignment.MunkresTrack(
                detections, self.personList, timestamp)

            for update in updates:
                self.personList[update].update()

        else:
            for detection in detections:
                self.personList.append(
                    KalmanFilter(
                        detection.x,
                        detection.y,
                        detection.orientation,
                        withTheta=detection.withTheta,
                        timestamp=timestamp))
