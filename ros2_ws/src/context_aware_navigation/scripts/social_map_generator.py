#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
from numba import jit
from scipy.ndimage import rotate
from multi_person_tracker_interfaces.msg import People, Person
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SocialMapGenerator(Node):

    def __init__(self, height, width, density):
        super().__init__('social_map_generator')
        self.width = width
        self.height = height
        self.density = density
        # %standard diviations %adjust to get different shapes
        self.sigmaFront = 2
        self.sigmaSide = 4/3
        self.sigmaBack = 1
        self.socialZones = []
        self.velocities = np.arange(0, 1.5, 0.1)
        self.center = ((width*density)/2, (height*density)/2)
        self.socialMap = None

        self.publisher_ = self.create_publisher(String, 'social_map', 10)
        self.cvBridge = CvBridge()
        self.people_sub = self.create_subscription(
            People,
            'people',
            self.people_callback,
            10)
        # tf listener stuff so we can transform people into there
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.people_sub  # prevent unused variable warning

    @jit(nopython=True)
    def people_callback(self, msg: People):
        # get the latest transform between the robot and the map
        try:
            t = self.tf_buffer.lookup_transform(
                "pr2/base_link",
                "map",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to map: {ex}')
            return

        social_map = np.zeros(
            (self.height*self.density, self.width*self.density), np.float32)

        for person in msg.people:
            # make the person position relative to the non rotating robot
            X = (person.position.x - t.transform.translation.x)*self.density
            Y = (person.position.y - t.transform.translation.y)*self.density
            if abs(X) < self.center[0] or abs(Y) < self.center[1]:
                X = np.floor(X + self.center[0])
                Y = np.floor(Y + self.center[1])
                # rotate LUT result for specific velocity
                vel = (person.velocity.x*person.velocity.x +
                       person.velocity.y*person.velocity.y)**0.5
                idx = (np.abs(self.velocities - vel)).argmin()
                social_zone = rotate(
                    self.socialZones(idx), person.position.z, reshape=True)

                (width, height) = np.shape(social_zone)
                self.socialMap[X-width:X+width, Y-height:Y +
                               height] = np.maximum(self.socialMap[X-width:X+width, Y-height:Y +
                                                                   height], social_zone)
                self.publisher_.publish(self.cvBridge.cv2_to_imgmsg(
                    self.socialMap, encoding="passthrough"))

    def initSocialZones(self):

        plotsize = 3
        # %create canvas
        # shift x by 1 meter
        x = np.arange(plotsize+1, plotsize+1, self.density)
        y = np.arange(plotsize, plotsize, self.density)

        for vel in self.velocities:
            self.socialZones.append(self.makeProxemicZone(
                0, 0, x, y, 0, self.sigmaFront+vel, self.sigmaSide, self.sigmaBack))

    @jit(nopython=True)
    def asymetricGaus(self, x=0, y=0, x0=0, y0=0, theta=0, sigmaFront=2, sigmaSide=4/3, sigmaBack=1) -> float:
        angle = np.mod(np.arctan2(y-y0, x-x0), 2*np.pi)+theta
        if (abs(angle) >= np.pi/2 and abs(angle) <= np.pi+np.pi/2):
            sigma = sigmaBack
        else:
            sigma = sigmaFront

        a = ((np.cos(theta) ** 2)/(2*sigma ** 2)) + \
            ((np.sin(theta) ** 2)/(2*sigmaSide ** 2))
        b = (np.sin(2*theta)/(4*sigma ** 2)) - \
            (np.sin(2*theta)/(4*sigmaSide ** 2))
        c = ((np.sin(theta) ** 2)/(2*sigma ** 2)) + \
            ((np.cos(theta) ** 2)/(2*sigmaSide ** 2))

        return np.exp(-(a*(x-x0) ** 2+2*b*(x-x0)*(y-y0)+c*(y-y0) ** 2))

    @jit(nopython=True)
    def makeProxemicZone(self, x0, y0, x, y, theta, sigmaFront, sigmaSide, sigmaBack) -> np.ndarray:
        social = np.zeros((len(x), len(y)), dtype=np.uint8)
        for i in range(len(x)):
            for j in range(len(y)):
                social[j, i] = self.thresholdCost(self.asymetricGaus(
                    x[i], y[j], x0, y0, theta, sigmaFront))
        return social

    @jit(nopython=True)
    def thresholdCost(self, cost: float) -> float:
        if cost > self.asymetricGaus(y=0.5):
            return 255
        if cost > self.asymetricGaus(y=1.0):
            return np.floor(255*self.asymetricGaus(y=1.0))
        if cost > self.asymetricGaus(y=1.5):
            return cost*255
        return 0


def main(args=None):
    rclpy.init(args=args)

    social_map_generator = SocialMapGenerator(3, 3, 0.05)
    rclpy.spin(social_map_generator)
    social_map_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
