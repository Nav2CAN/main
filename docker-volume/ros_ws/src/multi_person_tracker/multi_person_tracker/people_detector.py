import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from .tracking import PeopleTracker
import jetson_utils
from jetson_inference import poseNet
from jetson_utils import videoOutput, logUsage

import csv  # DC remove later

from .person_keypoints import *
from tf_transformations import quaternion_from_euler
from people_msgs.msg import People, Person
from geometry_msgs.msg import Point


class Detection:
    def __init__(self, x: float, y: float, orientation: float):
        self.x = x
        self.y = y
        self.orientation = orientation


class MultiPersonTracker(Node):
    def __init__(self, debug: bool = False, dt=0.1, keeptime=5):
        super().__init__('multi_person_tracker')
        self.create_timer(dt, self.time_callback)

        self.people_detector = self.PeopleDetector(n_cameras=2, debug=debug)
        self.people_tracker = PeopleTracker(debug=debug, keeptime=keeptime)

    def timer_callback(self):
        self.people_tracker.predict()

    def update(self):
        self.people_tracker.update()

    class PeopleDetector(Node):
        '''
        Class for pose estimation of a person using Nvidia jetson Orin implementation
        of PoseNet and passing messages using ROS2.
        The Class uses Intel Realsense messages on the ROS2 network as input for rgb and depth images
        '''

        def __init__(self, n_cameras: int = 2, publishPoseMsg: bool = True, debug: bool = True):
            super().__init__('people_detector')

            # DC For data collection
            self.peopleCount = 0
            self.imageCount = -1
            self.written = False
            self.debug = debug
            self.publishPoseMsg = publishPoseMsg
            self.cameras = []

            # Setup variables for PoseNet
            self.network = "resnet18-body"
            self.overlay = "links,keypoints,boxes"
            self.threshold = 0.3
            self.output_location = "/docker-volume/images"  # only needed for saving images

            # Initialising PoseNet and its output
            self.net = poseNet(
                self.network, ['people_detector.py'], self.threshold)
            self.output = videoOutput(self.output_location, argv=[
                'people_detector.py'])

            if n_cameras > 1:
                self.cameras = [self.Camera(namespace="camera"+str(i+1))
                                for i in range(n_cameras)]
            else:
                self.cameras = [self.Camera()]

            self.publisher_ = self.create_publisher(
                MarkerArray, 'poses', 10)

        def detect(self, camera):
            '''
            Perform pose estimation (with overlay)
            '''
            people = []

            if (camera.cudaimage != None) and isinstance(camera.depth, np.ndarray):
                poses = self.net.Process(
                    camera.cudaimage, overlay=self.overlay)
                kpPersons = camera.generatePeople(poses)
                people.append(kpPersons)
                detections = []
                for person in people:
                    detections.append(
                        Detection(person.x, person.y, person.orientation))
                MultiPersonTracker.update(
                    detections, camera.timestamp)  # Track people
                if len(kpPersons) != 0 and self.debug:
                    self.imageCount = self.saveImage(kpPersons)
                    self.peopleCount += len(kpPersons)
                    self.written = self.writing(kpPersons, camera.timestamp)

            if self.publishPoseMsg:
                self.publishPoseArrows(people)

            return people, camera.timestamp

        def publishPoseArrows(self, people):
            # Set the scale of the marker
            marker_array_msg = MarkerArray()
            for kpPersons in people:
                for i, kpPerson in enumerate(kpPersons):
                    # Set the pose of the marker
                    if (kpPerson.x and kpPerson.y and kpPerson.orientation):
                        quad = quaternion_from_euler(
                            0, 0, (2*np.pi + kpPerson.orientation if kpPerson.orientation < 0 else kpPerson.orientation))
                        marker = Marker()
                        marker.header.frame_id = "/"+self.namespace+"_link"
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.type = 0
                        marker.id = i
                        marker.pose.position.x = kpPerson.x
                        marker.pose.position.y = kpPerson.y
                        marker.pose.position.z = float(0)
                        marker.pose.orientation.x = float(quad[0])
                        marker.pose.orientation.y = float(quad[1])
                        marker.pose.orientation.z = float(quad[2])
                        marker.pose.orientation.w = float(quad[3])
                        marker.scale.x = 1.0
                        marker.scale.y = 0.1
                        marker.scale.z = 0.1

                        # Set the color
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        marker.color.a = 1.0
                        marker.frame_locked = False
                        marker_array_msg.markers.append(marker)
            self.publisher_.publish(marker_array_msg)

        def publishKeypointsMarker(self, people):
            marker_array_msg = MarkerArray()
            for kpPersons in people:
                for i, kpPerson in enumerate(kpPersons):
                    # Set the pose of the marker
                    if (kpPerson.x and kpPerson.y and kpPerson.orientation):
                        quad = quaternion_from_euler(
                            0, 0, (2*np.pi + kpPerson.orientation if kpPerson.orientation < 0 else kpPerson.orientation))
                        marker = Marker()
                        marker.header.frame_id = "/"+self.namespace+"_link"
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.type = 8
                        marker.id = i
                        marker.scale.x = .05
                        marker.scale.y = .05
                        marker.scale.z = .05
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        marker.color.a = 1.0
                        for kp in kpPerson.keypoints:
                            point = Point()
                            point.x = kp.x
                            point.y = kp.y
                            point.z = kp.z
                            marker.points.append(point)
                        marker_array_msg.markers.append(marker)
            self.publisher_.publish(marker_array_msg)

        def saveImage(self, personlist):
            self.imageCount += 1
            print("detected {:d} objects in image".format(len(personlist)))
            for person in personlist:
                print(person)
                print(person.keypoints)
                # print('Links', person.Links)
            self.output.Render(self.cudaimage)
            self.output.SetStatus("{:s} | Network {:.0f} FPS".format(
                self.network, self.net.GetNetworkFPS()))
            self.net.PrintProfilerTimes()

        def writing(self, personlist, timestamp):
            '''
            DC Function for writing csv file with person variables for captured images
            '''
            with open('SanityCheck.csv', mode='a') as csvfile:
                writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='"', quoting=csv.QUOTE_MINIMAL)

                if not self.written:
                    writer.writerow(['ImageID', 'Timestamp', 'PersonX', 'PersonY', 'Orientation',
                                    'left_shoulderX', 'left_shoulderY', 'left_shoulderZ',
                                     'right_shoulderX', 'right_shoulderY', 'right_shoulderZ'])
                    self.written = True
                for person in personlist:
                    left_shoulder = next(
                        (point for point in person.keypoints if point.ID == 5), None)
                    right_shoulder = next(
                        (point for point in person.keypoints if point.ID == 6), None)
                    if left_shoulder and right_shoulder:
                        writer.writerow([str(self.imageCount), str(timestamp), str(round(person.x, 3)), str(round(person.y, 3)), str(round(person.orientation, 3)),
                                        str(round(left_shoulder.x, 3)), str(
                                            round(left_shoulder.y, 3)), str(round(left_shoulder.z, 3)),
                                        str(round(right_shoulder.x, 3)), str(round(right_shoulder.y, 3)), str(round(right_shoulder.z, 3))])

        class Camera(Node):
            def __init__(self, namespace: str = "camera"):
                super().__init__(namespace+"_callback")
                self.rgb = None
                self.cudaimage = None
                self.depth = None
                self.namespace = namespace
                self.bridge = CvBridge()
                self.timestamp = None

                self.rgb_subscription = PeopleDetector.create_subscription(
                    Image,
                    '/' + namespace+'/color/image_raw',
                    self.rgb_callback,
                    10)
                self.guards

                self.depth_subscription = PeopleDetector.create_subscription(
                    Image,
                    '/' + namespace+'/aligned_depth_to_color/image_raw',
                    self.depth_callback,
                    10)
                self.guards

            def rgb_callback(self, msg):
                try:
                    self.rgb = self.bridge.imgmsg_to_cv2(
                        msg, desired_encoding='passthrough')
                    cudaimage = cv2.cvtColor(self.rgb, cv2.COLOR_BGRA2RGBA).astype(
                        np.float32)  # converting the image to a cuda compatible image
                    self.cudaimage = jetson_utils.cudaFromNumpy(cudaimage)
                    self.timestamp = self.get_clock().now().nanoseconds
                    # detect when new rgb immage is available
                    PeopleDetector.detect(camera=self)
                except:
                    pass

            def depth_callback(self, msg):
                try:
                    depth = self.bridge.imgmsg_to_cv2(
                        msg, desired_encoding='passthrough')
                    self.depth = np.array(depth, dtype=np.float32)*0.001
                    self.timestamp = self.get_clock().now().nanoseconds
                except:
                    pass

            def generatePeople(self, poses):
                '''
                Calculates the location of the person as X and Y coordinates along with the orientation of the person
                '''
                persons = []
                for pose in poses:
                    kpPerson = person_keypoint(pose.Keypoints, self.depth)
                    persons.append(kpPerson)
                return persons
