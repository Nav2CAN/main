import rclpy
import threading
import numpy as np
import cv2
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from cv_bridge import CvBridge

import jetson.inference
import jetson.utils
from jetson_inference import poseNet
from jetson_utils import videoOutput, logUsage

import csv # DC remove later

from .person_keypoints import *
from tf_transformations import quaternion_from_euler

class PoseEstimator(Node):
    '''
    Class for pose estimation of a person using Nvidia jetson implementation
    of PoseNet and passing messages using ROS2.
    The Class uses Intel Realsense messages on the ROS2 network as input for rgb and depth images
    '''

    def __init__(self):
        super().__init__('pose_estimator')

        # DC For data collection 
        self.peopleCount = 0
        self.imageCount = -1
        self.written = False

        # for working with images in ROS2
        self.bridge=CvBridge()
        self.rgb=None
        self.cudaimage=None
        self.depth=None
        self.poses=None
        self.publisher_ = self.create_publisher(Marker, 'keypoints', 10)
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10)
        
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10)
        self.rgb_subscription
        self.depth_subscription  # prevent unused variable warning

        # Setup variables for PoseNet
        self.network = "resnet18-body"
        self.overlay = "links,keypoints,boxes"
        self.threshold = 0.3
        self.output_location = "images" # only needed for saving images

        # Initialising PoseNet and its output
        self.net = poseNet(self.network, ['pose_estimator_node.py'], self.threshold)
        self.output = videoOutput(self.output_location, argv=['pose_estimator_node.py'])

    def rgb_callback(self, msg):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cudaimage = cv2.cvtColor(self.rgb, cv2.COLOR_BGRA2RGBA).astype(np.float32) #converting the image to a cuda compatible image
            self.cudaimage = jetson.utils.cudaFromNumpy(cudaimage)
        except:
            pass

    def depth_callback(self, msg):
        try:
            depth=self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth = np.array(depth, dtype=np.float32)*0.001
        except:
            pass


    def saveImage(self,img):
            self.imageCount += 1
            print("detected {:d} objects in image".format(len(self.poses)))
            for pose in self.poses:
                print(pose)
                print(pose.Keypoints)

                # print('Links', pose.Links)

            self.output.Render(img)
            self.output.SetStatus("{:s} | Network {:.0f} FPS".format(self.network, self.net.GetNetworkFPS()))
            self.net.PrintProfilerTimes()

    def publishPoseArrow(self, kpPerson:person_keypoint):
        marker=Marker()
        marker.header.frame_id = "/camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 0
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        # Set the pose of the marker
        if(kpPerson.x and kpPerson.y and kpPerson.orientation):
            quad=quaternion_from_euler(0,0,(2*np.pi + kpPerson.orientation if kpPerson.orientation < 0 else kpPerson.orientation))

            marker.pose.position.x = kpPerson.x
            marker.pose.position.y = kpPerson.y
            marker.pose.position.z = float(0)
            marker.pose.orientation.x = float(quad[0])
            marker.pose.orientation.y = float(quad[1])
            marker.pose.orientation.z = float(quad[2])
            marker.pose.orientation.w = float(quad[3])
            self.publisher_.publish(marker)

    def publishKeypointsMarker(self, kpPerson:person_keypoint):
        marker=Marker()
        marker.header.frame_id = "/camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 8
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = .05
        marker.scale.y = .05
        marker.scale.z = .05

        # Set the color
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
        self.publisher_.publish(marker)
    def getPersons(self):
        '''
        Calculates the location of the person as X and Y coordinates along with the orientation of the person
        '''
        persons = []
        for pose in self.poses:
            kpPerson=person_keypoint(pose.Keypoints, self.depth)
            self.publishPoseArrow(kpPerson=kpPerson)
            persons.append(kpPerson)
            
        return persons
    
    def writing(self, personlist):
        '''
        DC Function for writing csv file with person variables for captured images
        '''
        with open('SanityCheck.csv', mode = 'a') as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)            
            
            if not self.written:
                writer.writerow(['ImageID', 'PersonX', 'PersonY', 'Orientation',
                                 'left_shoulderX', 'left_shoulderY', 'left_shoulderZ',
                                'right_shoulderX', 'right_shoulderY', 'right_shoulderZ'])
                self.written = True
            for person in personlist:
                left_shoulder = next((point for point in person.keypoints if point.ID == 5), None)
                right_shoulder = next((point for point in person.keypoints if point.ID == 6), None)
                if left_shoulder and right_shoulder:
                    writer.writerow([str(self.imageCount), str(round(person.x,3)), str(round(person.y, 3)), str(round(person.orientation, 3)),
                                  str(round(left_shoulder.x, 3)), str(round(left_shoulder.y, 3)), str(round(left_shoulder.z, 3)),
                                  str(round(right_shoulder.x, 3)), str(round(right_shoulder.y, 3)), str(round(right_shoulder.z, 3))])


    def detectPoses(self):
        '''
        Perform pose estimation (with overlay)
        '''
        self.poses = self.net.Process(self.cudaimage, overlay=self.overlay)

        #TODO comment out when running node
        # print the pose results
        
        persons=self.getPersons()
        if len(persons)!=0:
            self.saveImage(self.cudaimage)
            self.peopleCount += len(persons) 
            self.writing(persons) 
          

def main(args=None):
    
    rclpy.init(args=args)
    pose_estimator = PoseEstimator()  # Start ROS2 node
    thread = threading.Thread(target=rclpy.spin, args=(pose_estimator, ), daemon=True)
    thread.start()

    rate = pose_estimator.create_rate(10)
    # initiate pose estimator object

    try:
        while rclpy.ok():
            rate.sleep()
            if(pose_estimator.cudaimage != None) and isinstance(pose_estimator.depth, np.ndarray): # Make sure an image has been captured
                pose_estimator.detectPoses()
                
            if pose_estimator.peopleCount == 1000: # DC for data collection run only until a certain amount of people have been detected
                break
            # rclpy.spin_once(pose_estimator)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()