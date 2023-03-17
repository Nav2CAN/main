# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import numpy as np
import cv2
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import sys
import argparse
import jetson.inference
import jetson.utils
from jetson_inference import poseNet
from jetson_utils import videoSource, videoOutput, logUsage

from dataclasses import dataclass

# @dataclass
# class Pose:
#     keypoints: list[float,float,float]
#     ID: list[float]
        
#     def __init__(self, keypoints):

#         self.keypoints=
#         self.ID=


class PoseEstimator(Node):


    def __init__(self):
        super().__init__('pose_estimator')

        # parser = argparse.ArgumentParser(description="Run pose estimation DNN on a video/image stream.", 
        #                         formatter_class=argparse.RawTextHelpFormatter, 
        #                         epilog=poseNet.Usage() + videoSource.Usage() + videoOutput.Usage() + logUsage())

        # parser.add_argument("input_URI", type=str, default="", nargs='?', help="URI of the input stream")
        # parser.add_argument("output_URI", type=str, default="", nargs='?', help="URI of the output stream")
        # parser.add_argument("--network", type=str, default="resnet18-body", help="pre-trained model to load (see below for options)")
        # parser.add_argument("--overlay", type=str, default="links,keypoints", help="pose overlay flags (e.g. --overlay=links,keypoints)\nvalid combinations are:  'links', 'keypoints', 'boxes', 'none'")
        # parser.add_argument("--threshold", type=float, default=0.15, help="minimum detection threshold to use") 

        self.bridge=CvBridge()
        self.rgb=None
        self.cudaimage=None
        self.depth=None
        self.poses=None

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

        self.network = "resnet18-body"
        self.overlay = "links,keypoints"
        self.threshold = 0.15
        self.output_location = "images"

        self.net = poseNet(self.network, ['pose_estimator_node.py'], self.threshold)
        # self.input = videoSource(self.rgb, argv=['pose_estimator_node.py'])
        self.output = videoOutput(self.output_location, argv=['pose_estimator_node.py'])

    def rgb_callback(self, msg):
        try:
            self.rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cudaimage = cv2.cvtColor(self.rgb, cv2.COLOR_BGRA2RGBA).astype(np.float32)#converting the image to a cuda compatible image
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
            print("detected {:d} objects in image".format(len(self.poses)))
            for pose in self.poses:
                # print(pose)
                # print(pose.Keypoints)
                print(pose.Keypoints[0].ID)
                print(pose.Keypoints[0].x)
                print(pose.Keypoints[0].y)

                # print('Links', pose.Links)

            self.output.Render(img)
            self.output.SetStatus("{:s} | Network {:.0f} FPS".format(self.network, self.net.GetNetworkFPS()))
            self.net.PrintProfilerTimes()


    # def determine3DPose(self):


    def detectPose(self):
        # perform pose estimation (with overlay)
        self.poses = self.net.Process(self.cudaimage, overlay=self.overlay)

        #TODO comment out when running node
        # print the pose results
        self.saveImage(self.cudaimage)

            
  

def main(args=None):
    
    rclpy.init(args=args)

    pose_estimator = PoseEstimator()

    try:
        while rclpy.ok():
            if(pose_estimator.cudaimage != None):
                pose_estimator.detectPose()
            rclpy.spin_once(pose_estimator)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
