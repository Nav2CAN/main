import rclpy
import os
from rclpy.node import Node
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs
import jetson_utils
from jetson_inference import poseNet
from jetson_utils import videoOutput, logUsage
import csv  # DC remove later

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

from .person_keypoints import *
from multi_person_tracker_interfaces.msg import People, Person
from .tracking import PeopleTracker


class Detection:
    def __init__(self, x: float, y: float, orientation: float):
        self.x = x
        self.y = y
        self.orientation = orientation


class MultiPersonTracker(Node):
    def __init__(self, debug: bool = False, publishPoseMsg: bool = True, dt=0.1, n_cameras=2, keeptime=5, target_frame: str = "map"):
        '''
        Class for pose estimation of a person using Nvidia jetson Orin implementation
        of PoseNet and passing messages using ROS2.
        The Class uses Intel Realsense messages on the ROS2 network as input for rgb and depth images
        :param debug: display debug messages in the console
        :param publishPoseMsg: publish marker arrows to the ROS2 network 
        :param dt: rate of prediction for the trackers
        :param n_cameras: number of publishing cameras on the ROS2 network
        :param keeptime: seconds to keep tracklets after last detection
        '''

        super().__init__('multi_person_tracker')
        self.create_timer(dt, self.timer_callback)
        self.people_tracker = PeopleTracker(debug=False, keeptime=keeptime)
        self.people_publisher = self.create_publisher(People, 'people', 10)
        self.people_arrow_publisher = self.create_publisher(
            MarkerArray, 'people_arrows', 10)
        self.publishPoseMsg = publishPoseMsg
        self.debug = debug
        self.target_frame = target_frame
        ### Variables for pose detection###
        self.peopleCount = 0
        self.imageCount = -1
        self.written = False
        self.cameras = []

        # Setup variables for PoseNet
        self.network = "resnet18-body"
        self.overlay = "links,keypoints,boxes"
        self.threshold = 0.3
        self.output_location = "/docker-volume/images"  # only needed for saving images

        # Initialising PoseNet and its output
        self.net = poseNet(
            self.network, [os.path.basename(__file__)], self.threshold)
        self.output = videoOutput(self.output_location, argv=[
            os.path.basename(__file__)])

        # Initialize camera objects with propper namespacing
        if n_cameras > 1:
            self.cameras = [self.tracker.Camera(self, namespace="camera"+str(i+1))
                            for i in range(n_cameras)]
        else:
            self.cameras = [self.Camera(self)]

    def timer_callback(self):
        # Publishes Tracker Ouput and predicts next state
        people = People()
        people.header.stamp = self.get_clock().now().to_msg()
        # TODO change when we have tf goodness
        people.header.frame_id = "/camera1_link"
        # TODO implement index and reliabílity
        for p in self.people_tracker.personList:
            person = Person()
            person.position.x = float(p.personX)
            person.position.y = float(p.personY)
            person.position.z = float(p.personTheta)
            person.velocity.x = float(p.personXdot)
            person.velocity.y = float(p.personYdot)
            person.velocity.z = float(p.personThetadot)
            people.people.append(person)

        self.people_publisher.publish(people)
        if self.publishPoseMsg:
            self.publishPoseArrows(self.people_tracker.personList)

        self.people_tracker.predict()

    def publishPoseArrows(self, people):
        # Set the scale of the marker
        marker_array_msg = MarkerArray()
        for i, person in enumerate(people):
            # Set the pose of the marker
            if (person.personX and person.personY and person.personTheta):
                quad = quaternion_from_euler(
                    0, 0, (2*np.pi + person.personTheta if person.personTheta < 0 else person.personTheta))
                marker = Marker()
                # TODO change when we have tf goodness
                marker.header.frame_id = "/camera1_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = 0
                marker.id = i
                marker.pose.position.x = float(person.personX)
                marker.pose.position.y = float(person.personY)
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
        self.people_arrow_publisher.publish(marker_array_msg)

    #########################################################################
    # THIS MIGHT BE USEFULL FOR THE REPORT SOMETIME BUT FOR NOW NOT NECESSARY#
    #########################################################################

    # def publishKeypointsMarker(self, people):
    #     marker_array_msg = MarkerArray()
    #     for kpPersons in people:
    #         for i, kpPerson in enumerate(kpPersons):
    #             # Set the pose of the marker
    #             if (kpPerson.x and kpPerson.y and kpPerson.orientation):
    #                 quad = quaternion_from_euler(
    #                     0, 0, (2*np.pi + kpPerson.orientation if kpPerson.orientation < 0 else kpPerson.orientation))
    #                 marker = Marker()
    #                 marker.header.frame_id = "/"+self.namespace+"_link"
    #                 marker.header.stamp = self.get_clock().now().to_msg()
    #                 marker.type = 8
    #                 marker.id = i
    #                 marker.scale.x = .05
    #                 marker.scale.y = .05
    #                 marker.scale.z = .05
    #                 marker.color.r = 0.0
    #                 marker.color.g = 1.0
    #                 marker.color.b = 0.0
    #                 marker.color.a = 1.0
    #                 for kp in kpPerson.keypoints:
    #                     point = Point()
    #                     point.x = kp.x
    #                     point.y = kp.y
    #                     point.z = kp.z
    #                     marker.points.append(point)
    #                 marker_array_msg.markers.append(marker)
    #     self.publisher_.publish(marker_array_msg)

    def detect(self, cudaImage, depthImage):
        '''
        Perform pose estimation (with overlay)
        '''
        if (cudaImage != None) and isinstance(depthImage, np.ndarray):
            poses = self.net.Process(
                cudaImage, overlay=self.overlay)
            return poses
        else:
            return None

    def saveImage(self, cudaImage):
        # render an image of the camera with a pose overlay
        self.imageCount += 1
        self.output.Render(cudaImage)
        self.output.SetStatus("{:s} | Network {:.0f} FPS".format(
            self.network, self.net.GetNetworkFPS()))
        self.net.PrintProfilerTimes()

    class Camera(object):
        def __init__(self, tracker_self, namespace: str = "camera", debug: bool = False):
            if self.debug:
                print("init camera")
            self.rgb = None
            self.cudaimage = None
            self.depth = None
            self.bridge = CvBridge()
            self.timestamp = None
            self.tracker = tracker_self
            self.debug = debug

            self.namespace = namespace
            self.tfFrame = self.namespace+"_link"
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self.tracker)

            # Initialize subscribers in tracker object for this camera
            self.rgb_subscription = self.tracker.create_subscription(
                Image,
                '/' + namespace+'/color/image_raw',
                self.rgb_callback,
                10)

            self.depth_subscription = self.tracker.create_subscription(
                Image,
                '/' + namespace+'/aligned_depth_to_color/image_raw',
                self.depth_callback,
                10)

        def rgb_callback(self, msg):
            try:
                # conversions
                self.rgb = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='passthrough')
                cudaimage = cv2.cvtColor(self.rgb, cv2.COLOR_BGRA2RGBA).astype(
                    np.float32)  # converting the image to a cuda compatible image
                self.cudaimage = jetson_utils.cudaFromNumpy(cudaimage)
                self.timestamp = self.tracker.get_clock().now().nanoseconds

                # detect poses when new rgb immage is available
                poses = self.tracker.detect(
                    self.cudaimage, self.depth)

                # generate 3D coordinates for all keypoints and calculate x,y,theta
                kpPersons = self.generatePeople(poses)

                # make detection objects
                detections = []
                trans = self.tf_buffer.lookup_transform(
                    self.tfFrame, self.tracker.target_frame, self.tracker.get_clock().now())
                pose = Pose()
                for person in kpPersons:
                    # transformation to target_frame
                    pose.position.x = person.x
                    pose.position.y = person.y
                    pose.position.z = 0
                    quad = quaternion_from_euler(0, 0, person.orientation)
                    pose.orientation = quad

                    pose = tf2_geometry_msgs.do_transform_pose(
                        pose, trans)

                    detections.append(
                        Detection(pose.position.x, pose.position.y, euler_from_quaternion(pose.orientation)))

                # Update tracker with new detections
                if len(detections) != 0:
                    self.tracker.people_tracker.update(
                        detections, self.timestamp)

                    # save image and make csv if required
                    if self.debug:
                        self.written = self.writing(kpPersons)
                        self.tracker.pose_detector.peopleCount += len(
                            kpPersons)
                        self.tracker.pose_detector.saveImage(self.cudaimage)
            except:
                if self.debug:
                    print(f"Exception on rgb_callback")

        def depth_callback(self, msg):
            # get and update depth image
            try:
                depth = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='passthrough')
                self.depth = np.array(depth, dtype=np.float32)*0.001
                # TODO Check do we actually want to update the timestamp
                self.timestamp = self.tracker.get_clock().now().nanoseconds
            except:
                if self.debug:
                    print(f"Exception on depth_callback")

        def generatePeople(self, poses):
            '''
            Calculates the location of the person as X and Y coordinates along with the orientation of the person
            '''
            persons = []
            for pose in poses:
                kpPerson = person_keypoint(pose.Keypoints, self.depth)
                persons.append(kpPerson)
            return persons

        def writing(self, personlist):
            '''
            Data collection function for writing csv file with person variables for captured images
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
                        writer.writerow([str(self.imageCount), str(self.timestamp), str(round(person.x, 3)), str(round(person.y, 3)), str(round(person.orientation, 3)),
                                        str(round(left_shoulder.x, 3)), str(
                                            round(left_shoulder.y, 3)), str(round(left_shoulder.z, 3)),
                                        str(round(right_shoulder.x, 3)), str(round(right_shoulder.y, 3)), str(round(right_shoulder.z, 3))])


def main(args=None):

    rclpy.init(args=args)

  # Start ROS2 node
    multi_person_tracker = MultiPersonTracker(
        dt=0.02, target_frame="camera1_link")
    rclpy.spin(multi_person_tracker)
    multi_person_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
