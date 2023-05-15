#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from multi_person_tracker_interfaces.msg import People, Person

import sys
import re

# from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

       
class StagePeople(Node):
    def __init__(self, num_hum):
        super().__init__('stage_people_publisher')
        self.num_hum = num_hum
        self.people_pub = []
        self.updated_people = []
        self.people_publisher = self.create_publisher(People, '/stage_people', 10)

        for person in range(self.num_hum):
            self.people_pub.append(0)
            self.updated_people.append(False)

        self.people_marker_sub = []
        for human_id in range(1,self.num_hum+1):
            name = 'human'+str(human_id)
            subscription = self.create_subscription(
            Odometry,
            f"/{name}/base_pose_ground_truth",
            self.people_callback,
            10)

            subscription

            self.people_marker_sub.append(subscription)
        

    def people_callback(self, msg):

        index = [int(num) for num in re.findall(r"\d+", msg.header.frame_id)]
        index = index[0] - 1

        # quad = [
        #             msg.orientation.x,
        #             msg.orientation.y,
        #             msg.orientation.z,
        #             msg.orientation.w]
        # angle = euler_from_quaternion(quad)[2]

        person = Person()
        person.position.x = float(msg.pose.pose.position.x)
        person.position.y = float(msg.pose.pose.position.y)
        person.position.z = float(3.14)
        # person.position.z = float(angle) #TODO add this instead


        self.people_pub[index] = person
        self.updated_people[index] = True
        if all(self.updated_people):
            people = People()
            people.header.stamp = msg.header.stamp
            for i, person in enumerate(self.people_pub):
                people.people.append(person)
                self.updated_people[i] = False # set false for net iteation
            
            self.people_publisher.publish(people)


if __name__ == '__main__':

    rclpy.init(args=sys.argv)
    nh = sys.argv[1]
    humans = StagePeople(num_hum=int(nh))
    rclpy.spin(humans)

    humans.destroy_node()
    rclpy.shutdown()