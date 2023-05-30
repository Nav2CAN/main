import rclpy
import os
from rclpy.node import Node
import threading
import numpy as np
import cv2
from cv_bridge import CvBridge
from tf_transformations import euler_from_quaternion, quaternion_about_axis
import tf2_ros
import tf2_geometry_msgs
import jetson_utils
from jetson_inference import poseNet
from jetson_utils import videoOutput
import csv  # DC remove later

from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, PointStamped

from .person_keypoints import *
from multi_person_tracker_interfaces.msg import People, Person
from .tracking import PeopleTracker, Detection
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy, ReliabilityPolicy

class MultiPersonTracker(Node):
    def __init__(self, publishPoseMsg: bool = True, publishKeypoints: bool = False, dt=0.1, n_cameras=2, newTrack=3, keeptime=5, target_frame: str = "map", debug: bool = False):
        '''
        Class for pose estimation of a person using Nvidia jetson Orin implementation
        of PoseNet and passing messages using ROS2.
        The Class uses Intel Realsense messages on the ROS2 network as input for rgb and depth images

        Parameters
        ----------
        publishPoseMsg: publish filtered marker arrows to the ROS2 network 
        publishKeypoints: publish non filtered keypoints as markers
        dt: rate of prediction for the trackers
        n_cameras: number of publishing cameras on the ROS2 network
        newTrack: meters distance at which detection is not assigned to tracklets and new ones are generated 
        keeptime: seconds to keep tracklets after last detection
        target_frame ouput tf_frame of the poses
        debug: display debug messages in the console
        '''

        super().__init__('multi_person_tracker')
        self.create_timer(dt, self.timer_callback)
        self.people_tracker = PeopleTracker(
            newTrack=newTrack, keeptime=keeptime, dt=dt, debug=debug)
        self.people_publisher = self.create_publisher(People, 'people', 10)
        self.people_arrow_publisher = self.create_publisher(
            MarkerArray, 'people_arrows', 10)
        self.people_keypoint_publisher = self.create_publisher(
            MarkerArray, 'people_keypoints', 10)
        self.publishPoseMsg = publishPoseMsg
        self.publishKeypointsMsg = publishKeypoints
        self.debug = debug
        self.target_frame = target_frame
        ### Variables for pose detection###
        self.peopleCount = 0
        self.imageCount = -1
        self.written = False
        self.cameras = []
        self.Orientations = []
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

        self.detectionMergingThreshold = 0.5
        # Initialize camera objects with propper namespacing
        if n_cameras > 1:
            self.cameras = [self.Camera(self, namespace="camera"+str(i+1))
                            for i in range(n_cameras)]
        else:
            self.cameras = [self.Camera(self)]

    def timer_callback(self):
        # Publishes Tracker Ouput and predicts next state
        people = People()
        people.header.stamp = self.get_clock().now().to_msg()
        # TODO change when we have tf goodness
        people.header.frame_id = self.target_frame
        # TODO implement index and reliabílity
        for p in self.people_tracker.tracklets:
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
            self.publishPoseArrows(self.people_tracker.tracklets)
        if self.publishKeypointsMsg:
            self.publishKeypoints(self.people_tracker.tracklets)
        self.people_tracker.predict(self.get_clock().now().nanoseconds)

    def publishPoseArrows(self, people):
        # Set the scale of the marker
        marker_array_msg = MarkerArray()

        for i, person in enumerate(people):
            # Set the pose of the marker
            if (person.personX and person.personY and person.personTheta):
                quad = quaternion_about_axis(person.personTheta, (0, 0, 1))
                marker = Marker()
                marker.header.frame_id = self.target_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = 0
                marker.id = i
                marker.pose.position.x = float(person.personX)
                marker.pose.position.y = float(person.personY)
                marker.pose.position.z = float(0)
                marker.pose.orientation.x = quad[0]
                marker.pose.orientation.y = quad[1]
                marker.pose.orientation.z = quad[2]
                marker.pose.orientation.w = quad[3]
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

    def publishKeypoints(self, people):
        # Set the scale of the marker
        marker_array_msg = MarkerArray()
        for i, person in enumerate(people):
            # Set the pose of the marker
            if (person.personX and person.personY and person.personTheta and len(person.keypoints)):
                # Set the pose of the marker
                marker = Marker()
                marker.header.frame_id = self.target_frame
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
                for kp in person.keypoints:
                    marker.points.append(kp.point)
                marker_array_msg.markers.append(marker)
        self.people_keypoint_publisher.publish(marker_array_msg)

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
        def __init__(self, tracker_self, namespace: str = "camera"):

            self.rgb = None
            self.cudaimage = None
            self.depth = None
            self.bridge = CvBridge()
            self.timestamp = None
            self.tracker = tracker_self
            self.debug = self.tracker.debug
            if self.debug:
                print("init camera")
            self.qos_profile = QoSProfile(
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=5)
            self.namespace = namespace
            self.tfFrame = self.namespace+"_color_frame"#TODO check if this is supposed to be "aligned_depth_to_color_frame"
            self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=5.0))
            self.tf_listener = tf2_ros.TransformListener(
                self.tf_buffer, self.tracker, spin_thread = True)

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
                if poses:
                    kpPersons = self.generatePeople(poses)
                    z = np.array([[complex(p.x, p.y) for p in kpPersons]])
                    popCounter=0
                    distanceMatrix=abs(z.T-z)
                    distanceMatrix = np.where(np.logical_and(0 < distanceMatrix, distanceMatrix < self.tracker.detectionMergingThreshold))
                    for i,detection in enumerate(distanceMatrix[0]):
                        kpPersons[detection-popCounter].x = (kpPersons[detection-popCounter].x + kpPersons[distanceMatrix[1][i]-popCounter].x)/2
                        kpPersons[detection-popCounter].y = (kpPersons[detection-popCounter].y + kpPersons[distanceMatrix[1][i]-popCounter].y)/2
                        kpPersons[detection-popCounter].orientation = (kpPersons[detection-popCounter].orientation + kpPersons[distanceMatrix[1][i]-popCounter].orientation)/2
                        kpPersons.pop(distanceMatrix[1][i]-popCounter)
                        popCounter+=1
                        if self.debug: print("removed double detection")
                    # make detection objects
                    detections = []
                    trans = None
                    try:
                        # self.tf_buffer.waitForTransform(self.tfFrame,self.tracker.target_frame, self.tracker.get_clock().now(), rclpy.time.Duration(seconds=5.0))
                        trans = self.tf_buffer.lookup_transform(
                            self.tracker.target_frame, self.tfFrame, self.tracker.get_clock().now(),timeout=rclpy.time.Duration(seconds=0.5))
                    except Exception as e:
                        print(e)
                    if trans:
                        pose = Pose()
                        for person in kpPersons:
                            try:
                                if self.tracker.publishKeypoints:
                                    keypoints = []
                                    for kp in person.keypoints:
                                        if kp.x and kp.y and kp.z:
                                            point = PointStamped()
                                            point.point.x = float(kp.x)
                                            point.point.y = float(kp.y)
                                            point.point.z = float(kp.z)
                                            point = tf2_geometry_msgs.do_transform_point(
                                                point, trans)
                                            keypoints.append(point)
                                # transformation to target_frame
                                pose.position.x = float(person.x)
                                pose.position.y = float(person.y)
                                pose.position.z = float(0.0)
                                angle = person.orientation if person.orientation < np.pi else person.orientation-2*np.pi
                                quad = quaternion_about_axis(
                                    person.orientation, (0, 0, 1))
                                pose.orientation.x = quad[0]
                                pose.orientation.y = quad[1]
                                pose.orientation.z = quad[2]
                                pose.orientation.w = quad[3]
                                pose = tf2_geometry_msgs.do_transform_pose(
                                    pose, trans)
                                quad = [
                                    pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z,
                                    pose.orientation.w,
                                ]
                                angle = euler_from_quaternion(quad)[2]
                                angle = angle if angle > 0 else angle+2*np.pi


                                if self.tracker.publishKeypoints:
                                    detections.append(
                                        Detection(pose.position.x, pose.position.y, angle, person.withTheta, keypoints))
                                else:
                                    detections.append(
                                        Detection(pose.position.x, pose.position.y, angle, person.withTheta))
                            except np.linalg.LinAlgError:
                                pass
                        # Update tracker with new detections
                        if len(detections):
                            self.tracker.people_tracker.update(
                                detections, self.timestamp)

                            # save image and make csv if required
                            if self.debug:
                                # self.writing(kpPersons)
                                self.tracker.peopleCount += len(
                                    kpPersons)
                                self.tracker.saveImage(self.cudaimage)
            except Exception as e:
                if self.debug:
                    print(f"Exception on rgb_callback")
                    print(e)

        def depth_callback(self, msg):
            # get and update depth image
            try:
                depth = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='passthrough')
                self.depth = np.array(depth, dtype=np.float32)*0.001
                # TODO Check do we actually want to update the timestamp
                self.timestamp = self.tracker.get_clock().now().nanoseconds
            except Exception as e:
                if self.debug:
                    print(f"Exception on depth_callback")
                    print(e)

        def generatePeople(self, poses):
            '''
            Calculates the location of the person as X and Y coordinates along with the orientation of the person
            '''
            persons = []
            for pose in poses:
                kpPerson = person_keypoint(pose.Keypoints, self.depth)
                if kpPerson.x != None and kpPerson.y != None:
                    persons.append(kpPerson)
            return persons

        def writing(self, orientation):
            '''
            Data collection function for writing csv file with person variables for captured images
            '''
            with open('Measurement.csv', mode='a') as csvfile:
                writer = csv.writer(csvfile, delimiter=',',
                                    quotechar='"', quoting=csv.QUOTE_MINIMAL)

                if not self.tracker.written:
                    writer.writerow(['Orientation'])

                self.tracker.written = True
                writer.writerow([str(round(orientation, 3))])


def main(args=None):

    rclpy.init(args=args)
  # Start ROS2 node
    multi_person_tracker = MultiPersonTracker(publishKeypoints=False,
                                              dt=0.02, target_frame="camera_link", debug=False)
    rclpy.spin(multi_person_tracker)
    multi_person_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
