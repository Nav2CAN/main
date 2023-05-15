#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys

# from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

       
class Odometer(Node):
    def __init__(self):
        super().__init__('odometer')
        self.sub = self.create_subscription(
            Odometry,
            "odom",
            self.callback,
            10)

        self.total_dist = 0
        self.prev_x = 0
        self.prev_y = 0
        self.first_run = True


    def callback(self, msg):

        if self.first_run:
            self.prev_x = float(msg.pose.pose.position.x)
            self.prev_y = float(msg.pose.pose.position.y)
            self.first_run = False

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)

        movement = ((x - self.prev_x)**2 + (y - self.prev_y)**2)**0.5

        self.total_dist += movement
        print(self.total_dist)
        
        self.prev_x = float(msg.pose.pose.position.x)
        self.prev_y = float(msg.pose.pose.position.y)
        



if __name__ == '__main__':

    rclpy.init(args=sys.argv)
    odometer = Odometer()
    rclpy.spin(odometer)

    odometer.destroy_node()
    rclpy.shutdown()