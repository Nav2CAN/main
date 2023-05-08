#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
from scipy.ndimage import rotate
from multi_person_tracker_interfaces.msg import People
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from context_aware_navigation.asymetricGausian import *


class SocialMapGenerator(Node):

    def __init__(self, height, width, density):
        super().__init__('social_map_generator')
        self.width = width
        self.height = height
        self.density = density
        self.socialCostSize = 4
        # %standard diviations %adjust to get different shapes
        self.sigmaFront = 2
        self.sigmaSide = 4/3
        self.sigmaBack = 1
        self.velocities = np.array([0])
        self.socialZones = initSocialZones(
            self.density, 2, 4/3, 1, self.velocities, self.socialCostSize)

        self.center = ((self.width*self.density)/2,
                       (self.height*self.density)/2)
        self.socialMap = None

        self.publisher_ = self.create_publisher(Image, 'social_map', 10)
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

    def people_callback(self, msg: People):
        # get the latest transform between the robot and the map
        try:
            t = self.tf_buffer.lookup_transform(
                "camera1_link",
                "camera1_link",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to map: {ex}')
            return

        self.socialMap = np.zeros(
            (round(self.height/self.density), round(self.width/self.density)), np.float32)
        self.center = (np.shape(self.socialMap)[
                       0]/2, np.shape(self.socialMap)[1]/2)  # [px]
        for person in msg.people:
            # make the person position relative to the non rotating robot
            X = int(np.floor((person.position.x - t.transform.translation.x) /
                             self.density))  # [px]
            Y = -int(
                np.floor((person.position.y - t.transform.translation.y) / self.density))
            if abs(X) < self.center[0] or abs(Y) < self.center[1]:
                # transform relative to the top left corner of the map
                X = int(np.floor((X + self.center[0])))
                Y = int(np.floor((Y + self.center[1])))
                # rotate LUT result for specific velocity
                vel = (person.velocity.x*person.velocity.x +
                       person.velocity.y*person.velocity.y)**0.5
                idx = (np.abs(self.velocities - vel)).argmin()
                social_zone = rotate(
                    self.socialZones[idx], np.rad2deg(person.position.z), reshape=True)

                (width, height) = np.shape(social_zone)
                width = int(np.floor(width/2))
                height = int(np.floor(height/2))

                minx = max(0, X-width)
                maxx = min(np.shape(self.socialMap)[0], X+width)
                miny = max(0, Y-height)
                maxy = min(np.shape(self.socialMap)[1], Y+width)

                roi = self.socialMap[miny:maxy, minx:maxx]

                sminx = width - min(width, X)
                sminy = height - min(height, Y)
                smaxx = width + min(width, np.shape(self.socialMap)[0]-X)
                smaxy = height + min(height, np.shape(self.socialMap)[1]-Y)

                social_zone = social_zone[sminy:smaxy, sminx:smaxx]
                self.socialMap[miny:maxy, minx:maxx] = np.maximum(
                    roi, social_zone)
        self.publisher_.publish(self.cvBridge.cv2_to_imgmsg(
            self.socialMap, encoding="passthrough"))


def main(args=None):
    rclpy.init(args=args)

    social_map_generator = SocialMapGenerator(15, 15, 0.05)
    rclpy.spin(social_map_generator)
    social_map_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
