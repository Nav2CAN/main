#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from tf_transformations import quaternion_about_axis, euler_from_quaternion

import numpy as np
from scipy.ndimage import rotate
from multi_person_tracker_interfaces.msg import People
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from context_aware_navigation.asymetricGausian import *


class SocialMapGenerator(Node):

    def __init__(self, height, width, density):
        super().__init__('social_map_generator')
        self.width = width
        self.height = height
        self.density = density  # px/m
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
        # TODO assign correct tf_frames

        try:
            t = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
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
            # transform person into map frame
            personPose = PoseStamped()
            personPose.header = msg.header
            personPose.pose.position.x = person.position.x
            personPose.pose.position.y = person.position.y
            personPose.pose.position.z = 0.0
            quad = quaternion_about_axis(person.position.z, (0, 0, 1))
            personPose.pose.orientation.x = quad[0]
            personPose.pose.orientation.y = quad[1]
            personPose.pose.orientation.z = quad[2]
            personPose.pose.orientation.w = quad[3]
            personOut = self.tf_buffer.transform(
                personPose, "map", rclpy.time.Duration(seconds=5.0))
            # make the person position relative to the non rotating robot

            X = int(np.floor((personOut.pose.position.x - t.transform.translation.x) /
                             self.density))  # [px]
            Y = -int(
                np.floor((personOut.pose.position.y - t.transform.translation.y) / self.density))
            if abs(X) < self.center[0] or abs(Y) < self.center[1]:
                # transform relative to the top left corner of the map
                X = int(np.floor((X + self.center[0])))
                Y = int(np.floor((Y + self.center[1])))

                quad = [
                    personOut.pose.orientation.x,
                    personOut.pose.orientation.y,
                    personOut.pose.orientation.z,
                    personOut.pose.orientation.w,
                ]
                social_zone = rotate(
                    self.socialZones[0], np.rad2deg(euler_from_quaternion(quad)[2]), reshape=True)

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
            self.socialMap, encoding="passthrough", header=msg.header))


def main(args=None):
    rclpy.init(args=args)

    social_map_generator = SocialMapGenerator(15, 15, 0.1)
    rclpy.spin(social_map_generator)
    social_map_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
