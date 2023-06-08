#!/usr/bin/env python3
from pathlib import Path

import torch
import numpy as np
import cv2

from .models.experimental import attempt_load
from .utils.datasets import letterbox
from .utils.general import check_img_size, non_max_suppression, \
    scale_coords, xyxy2xywh, set_logging
from .utils.torch_utils import select_device, time_synchronized, TracedModel

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from multi_person_tracker_interfaces.msg import BoundingBoxes, BoundingBox
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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

    def __init__(self, weights='/ros_ws/src/interaction_detection/interaction_detection/yolov7-ContextNav.pt',
                 img_size=320, map_size=15,
                 trace=True, augment=False, conf_thres=0.25, iou_thres=0.45,
                 classes=None, agnostic_nms=False, device=''):

        super().__init__('context_aware_detector')
        self.weights, self.imgsz, self.trace = weights, img_size, trace
        self.augment, self.conf_thres, self.iou_thres = augment, conf_thres, iou_thres
        self.classes, self.agnostic_nms = classes, agnostic_nms
        self.map_size = map_size

        # Initialize
        set_logging()
        self.device = select_device(device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA

        # Load model
        self.model = attempt_load(
            self.weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(
            self.imgsz, s=self.stride)  # check img_size

        if self.trace:
            self.model = TracedModel(self.model, self.device, self.imgsz)

        if self.half:
            self.model.half()  # to FP16

        # Get names and colors
        self.names = self.model.module.names if hasattr(
            self.model, 'module') else self.model.names

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(
                self.device).type_as(next(self.model.parameters())))  # run once
        self.old_img_w = self.old_img_h = self.imgsz
        self.old_img_b = 1

        # ROS2 subscriber and publisher setup
        self.interaction_publisher = self.create_publisher(
            BoundingBoxes, '/interaction_bb', 10)

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/social_map',
            self.social_zone_callback,
            10)
        self.subscription  # prevent unused variable warning
        # tf listener stuff so we can transform people into there
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        print("DONE INITIALIZING INTERACTION DETECTOR")

    def social_zone_callback(self, msg):
        try:
            t = self.tf_buffer.lookup_transform_full(
                target_frame="map",
                target_time=rclpy.time.Time(),
                source_frame=msg.header.frame_id,
                source_time=msg.header.stamp,
                fixed_frame="map")
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to map: {ex}')
            return

        try:

            im0s = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            # Convert image to rgb for YOLOv7
            im0s = cv2.cvtColor(im0s, cv2.COLOR_GRAY2RGB)
            self.timestamp = self.get_clock().now().nanoseconds

            assert im0s is not None, 'Image Not Found '
            img = letterbox(im0s, self.imgsz, stride=self.stride)[0]

            # Convert
            # BGR to RGB, to 3x416x416
            img = img[:, :, ::-1].transpose(2, 0, 1)
            img = np.ascontiguousarray(img)

            detections = self.detect(im0s, img) # class_ID, x, y, w, h, confi
            
            boundingBoxes = BoundingBoxes()
            boundingBoxes.header.stamp = self.get_clock().now().to_msg()
            boundingBoxes.header.frame_id = "map"
            for detection in detections:
                
                x = float(detection[1])
                y = float(detection[2])
                w = float(detection[3])
                h = float(detection[4])
                if detection[0] != None:

                    # put center value in middle of map and convert to meters
                    x = (x - 0.5) * self.map_size
                    # put center value in middle of map and convert to meters
                    y = (y - 0.5) * self.map_size

                    w = w * self.map_size  # transform to meters
                    h = h * self.map_size  # transform to meters

                    # transform center coordinates into /map frame
                    # orientation does not matter since the two maps are x,y-colinear

                    bb = BoundingBox()
                    bb.center_x = x +t.transform.translation.x 
                    bb.center_y = -y + t.transform.translation.y
                    bb.width = w
                    bb.height = h
                    boundingBoxes.boundingboxes.append(bb)
            self.interaction_publisher.publish(boundingBoxes)
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
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)
        t3 = time_synchronized()

        # Process detections
        for i, det in enumerate(pred):  # detections per image

            s, im0 = '', im0

            # normalization gain whwh
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    # add to string
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "
                
                detections= []
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) /
                            gn).view(-1).tolist()  # normalized xywh
                    line = (cls, *xywh, conf)
                    output = (('%g ' * len(line)).rstrip() % line).split(' ')

                    class_ID = output[0]
                    x = output[1]
                    y = output[2]
                    w = output[3]
                    h = output[4]
                    confi = output[5]
                    detections.append(output)

                    # print(f"Class = {class_ID}, x = {x}, y = {y}, width = {w}, height = {h}, conf = {confi}") # Remove after ROS implementation

            # Print time (inference + NMS)
                # print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')
                return detections #class_ID, x, y, w, h, confi
            # TODO handle output when no detection
            return None, None, None, None, None, None


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
