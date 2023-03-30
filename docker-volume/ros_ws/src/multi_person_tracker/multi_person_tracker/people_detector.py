import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

import jetson_utils
from jetson_inference import poseNet
from jetson_utils import videoOutput, logUsage

import csv # DC remove later

from .person_keypoints import *
from tf_transformations import quaternion_from_euler

class PeopleDetector(Node):
    '''
    Class for pose estimation of a person using Nvidia jetson Orin implementation
    of PoseNet and passing messages using ROS2.
    The Class uses Intel Realsense messages on the ROS2 network as input for rgb and depth images
    '''

    def __init__(self, n_cameras: int = 2, publishPoseMsg: bool = True, debug: bool =True):
        super().__init__('people_detector')

        # DC For data collection 
        self.peopleCount = 0
        self.imageCount = -1
        self.written = False
        self.debug = debug
        self.publishPoseMsg = publishPoseMsg
        self.cameras=[]

        # Setup variables for PoseNet
        self.network = "resnet18-body"
        self.overlay = "links,keypoints,boxes"
        self.threshold = 0.3
        self.output_location = "/docker-volume/images" # only needed for saving images

        # Initialising PoseNet and its output
        self.net = poseNet(self.network, ['people_detector.py'], self.threshold)
        self.output = videoOutput(self.output_location, argv=['people_detector.py'])

        if n_cameras>1:
            self.cameras = [Camera(self.default_callback_group,namespace="camera"+str(i+1)) for i in range(n_cameras)]
        else:
            self.cameras = [Camera(self.default_callback_group)]

    def detect(self):
        '''
        Perform pose estimation (with overlay)
        '''
        people=[]
        timestamps=[]
        for camera in self.cameras:
            if(camera.cudaimage != None) and isinstance(camera.depth, np.ndarray):
                poses = self.net.Process(camera.cudaimage, overlay=self.overlay)
                kpPersons=camera.generatePeople(poses)
                people.append(kpPersons)
                timestamps.append(camera.timestamp)
                if self.publishPoseMsg:
                    camera.publishPoseArrows(kpPersons)
                if len(kpPersons)!=0 and self.debug:
                    self.imageCount=camera.saveImage(kpPersons,self.imageCount,self.output,self.net,self.network)
                    self.peopleCount += len(kpPersons) 
                    self.written=camera.writing(kpPersons,self.written,self.imageCount) 

        return people, timestamps
    


class Camera(Node):
    def __init__(self,callback_group, namespace: str= "camera"):
        super().__init__(namespace+"_callback")
        self.rgb=None
        self.cudaimage=None
        self.depth=None
        self.namespace = namespace
        self.bridge=CvBridge()
        self.timestamp=None
        self.publisher_ = self.create_publisher(MarkerArray, 'poses', 10,callback_group=callback_group)
        
        self.rgb_subscription = self.create_subscription(
            Image,
            '/'+ namespace+'/color/image_raw',
            self.rgb_callback,
            10,callback_group=callback_group)
        self.guards 
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/'+ namespace+'/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10,callback_group=callback_group)
        self.guards 
        
        # self.rgb_subscription
        # self.depth_subscription 

    def rgb_callback(self, msg):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cudaimage = cv2.cvtColor(self.rgb, cv2.COLOR_BGRA2RGBA).astype(np.float32) #converting the image to a cuda compatible image
            self.cudaimage = jetson_utils.cudaFromNumpy(cudaimage)
            self.timestamp=self.get_clock().now().nanoseconds
        except:
            pass

    def depth_callback(self, msg):
        try:
            depth=self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth = np.array(depth, dtype=np.float32)*0.001
            self.timestamp=self.get_clock().now().nanoseconds
        except:
            pass

    def generatePeople(self, poses):
        '''
        Calculates the location of the person as X and Y coordinates along with the orientation of the person
        '''
        persons = []
        for pose in poses:
            kpPerson=person_keypoint(pose.Keypoints, self.depth)
            persons.append(kpPerson)
        return persons
    
    def publishPoseArrows(self, kpPersons):
        # Set the scale of the marker
        marker_array_msg = MarkerArray()
        for kpPerson in kpPersons:
            # Set the pose of the marker
            if(kpPerson.x and kpPerson.y and kpPerson.orientation):
                quad=quaternion_from_euler(0,0,(2*np.pi + kpPerson.orientation if kpPerson.orientation < 0 else kpPerson.orientation))
                marker = Marker()
                marker.header.frame_id = "/"+self.namespace+"_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = 0
                marker.id = 0
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
    def publishKeypointsMarker(self, kpPersons: List[person_keypoint]):
        for kpPerson in kpPersons:
            marker_array_msg = MarkerArray()
            # Set the pose of the marker
            if(kpPerson.x and kpPerson.y and kpPerson.orientation):
                quad=quaternion_from_euler(0,0,(2*np.pi + kpPerson.orientation if kpPerson.orientation < 0 else kpPerson.orientation))
                marker = Marker()
                marker.header.frame_id = "/"+self.namespace+"_link"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.type = 8
                marker.id = 0
                marker.scale.x = .05
                marker.scale.y = .05
                marker.scale.z = .05
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                for kp in kpPerson.keypoints:
                    point=Point()
                    point.x=kp.x
                    point.y=kp.y
                    point.z=kp.z
                    marker.points.append(point)
                marker_array_msg.markers.append(marker)
        self.publisher_.publish(marker_array_msg)


    def saveImage(self,poses,imageCount, output,net,network):
        imageCount += 1
        print("detected {:d} objects in image".format(len(poses)))
        for pose in poses:
            print(pose)
            print(pose.keypoints)
            # print('Links', pose.Links)
        output.Render(self.cudaimage)
        output.SetStatus("{:s} | Network {:.0f} FPS".format(network, net.GetNetworkFPS()))
        net.PrintProfilerTimes()
        return imageCount


    def writing(self, personlist, written, imageCount):
        '''
        DC Function for writing csv file with person variables for captured images
        '''
        with open('SanityCheck.csv', mode = 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)            
            
            if not written:
                writer.writerow(['ImageID','Timestamp', 'PersonX', 'PersonY', 'Orientation',
                                 'left_shoulderX', 'left_shoulderY', 'left_shoulderZ',
                                'right_shoulderX', 'right_shoulderY', 'right_shoulderZ'])
                written = True
            for person in personlist:
                left_shoulder = next((point for point in person.keypoints if point.ID == 5), None)
                right_shoulder = next((point for point in person.keypoints if point.ID == 6), None)
                if left_shoulder and right_shoulder:
                    writer.writerow([str(imageCount), str(self.timestamp), str(round(person.x,3)), str(round(person.y, 3)), str(round(person.orientation, 3)),
                                  str(round(left_shoulder.x, 3)), str(round(left_shoulder.y, 3)), str(round(left_shoulder.z, 3)),
                                  str(round(right_shoulder.x, 3)), str(round(right_shoulder.y, 3)), str(round(right_shoulder.z, 3))])
        return written
