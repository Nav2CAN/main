#!/usr/bin/env python3
from pathlib import Path

import torch
import numpy as np
import cv2

from models.experimental import attempt_load
from context_aware_navigation.utils.datasets import letterbox
from context_aware_navigation.utils.general import check_img_size, non_max_suppression, \
    scale_coords, xyxy2xywh, set_logging

from context_aware_navigation.utils.torch_utils import select_device, time_synchronized, TracedModel

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class Detector(Node):
    '''
    Class interaction detection of people using Nvidia jetson Orin in ROS2
    The Class uses images of social zones to detect interactions and publishes
    the global map location of the interaction from the detected bounding box
    ----------
    weights: the trained weights for detecting interaction
    img_size: the width of the image for input
    trace: choose whether the model is traced
    augment: choose if the images are augmented
    conf_thres: choose the confidence threshold for detection output
    iou_thres: choose the intersection over union threshold for NMS
    classes: change the name of classes if different from the training
    agnostic_nms: choose if the classes affect the IOU NMS
    device: choose compute device
    '''
    def __init__(self, weights = '/home/jonathan/repositories/master-thesis/ros2_ws/src/context_aware_navigation/yolov7-ContextNav.pt',
                  img_size = 320, map_size = 15,
                  trace = True, augment = False, conf_thres = 0.25, iou_thres = 0.45,
                 classes = None, agnostic_nms = False, device = ''):
        self.weights, self.imgsz, self.trace =  weights, img_size, trace
        self.augment, self.conf_thres, self.iou_thres = augment, conf_thres, iou_thres
        self.classes, self.agnostic_nms = classes, agnostic_nms
        self.map_size = map_size
        
        # Initialize
        set_logging()
        self.device = select_device(device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check img_size

        if self.trace:
            self.model = TracedModel(self.model, self.device, self.imgsz)

        if self.half:
            self.model.half()  # to FP16

        # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once
        self.old_img_w = self.old_img_h = self.imgsz
        self.old_img_b = 1

        #ROS2 subscriber and publisher setup
        # self.interaction_publisher = self.create_publisher(Interaction, 'interaction', 10) #TODO create interaction msg
        
        self.bridge = CvBridge()
        super().__init__('context_aware_detector')
        self.subscription = self.create_subscription(
                Image,
                '/social_map',
                self.social_zone_callback,
                10)
        self.subscription  # prevent unused variable warning



    def social_zone_callback(self, msg):
        try:
            im0s = self.bridge.imgmsg_to_cv2(
                        msg, desired_encoding='passthrough')
            im0s = cv2.cvtColor(im0s,cv2.COLOR_GRAY2RGB) # Convert image to rgb for YOLOv7
            self.timestamp = self.get_clock().now().nanoseconds

            assert im0s is not None, 'Image Not Found '
            img = letterbox(im0s, self.imgsz, stride=self.stride)[0]
            
            # Convert
            img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
            img = np.ascontiguousarray(img)

            class_ID, x, y, w, h, confi = self.detect(im0s, img)
            tf = (0,0) # TODO add tf functionality for transforming map

            x = tf[0] + (x - 0.5) * self.map_size # put center value in middle of map and convert to meters
            y = tf[1] +(y - 0.5)*self.map_size # put center value in middle of map and convert to meters
            
            w = w*self.map_size # transform to meters
            h = h*self.map_size # transform to meters



            print(self.result) # TODO publish values instead
            # msg = Interaction_msg
            # msg.data = class_ID, x, y, w, h, confi
            # self.interaction_publisher.publish(msg)

        except Exception as e:
                print(f"Exception on social_zone_callback")
                print(e)


    def detect(self, im0, img):
   
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Warmup
            if self.device.type != 'cpu' and (self.old_img_b != img.shape[0] or self.old_img_h != img.shape[2] or self.old_img_w != img.shape[3]):
                self.old_img_b = img.shape[0]
                self.old_img_h = img.shape[2]
                self.old_img_w = img.shape[3]
                for i in range(3):
                    self.model(img, augment=self.augment)[0]

            # Inference
            t1 = time_synchronized()
            with torch.no_grad():   # Calculating gradients would cause a GPU memory leak
                pred = self.model(img, augment=self.augment)[0]
            t2 = time_synchronized()

            # Apply NMS
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)
            t3 = time_synchronized()

            # Process detections
            for i, det in enumerate(pred):  # detections per image
            
                s, im0 = '', im0

                gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf)
                        output = (('%g ' * len(line)).rstrip() % line).split(' ')
                        
                        class_ID = output[0]
                        x = output[1]
                        y = output[2]
                        w = output[3]
                        h = output[4]
                        confi = output[5]

                        #print(f"Class = {class_ID}, x = {x}, y = {y}, width = {w}, height = {h}, conf = {confi}") # Remove after ROS implementation

                # Print time (inference + NMS)
                    #print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')
                    return class_ID, x, y, w, h, confi
                return None


def main(args=None):

    rclpy.init(args=args)
  # Start ROS2 node
    with torch.no_grad():
        detector = Detector()
        rclpy.spin(detector)
    
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
