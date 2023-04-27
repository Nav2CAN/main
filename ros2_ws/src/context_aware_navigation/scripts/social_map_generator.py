import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
from scipy.ndimage import rotate
from multi_person_tracker_interfaces.msg import People, Person


class SocialMapGenerator(Node):

    def __init__(self, height, width, density):
        super().__init__('social_map_generator')
        self.width = width
        self.height = height
        self.density = density
        self.center = ((width*density)/2, (height*density)/2)
        self.people_sub = self.create_subscription(
            People,
            'people',
            self.people_callback,
            10)
        # tf listener stuff so we can transform people into there
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.people_sub  # prevent unused variable warning

    def people_callback(self, msg: People):
        # get the latest transform between the robot and the map
        try:
            t = self.tf_buffer.lookup_transform(
                "pr2/base_link",
                "map",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to map: {ex}')
            return

        social_map = np.zeros(
            (self.height*self.density, self.width*self.density), np.float32)

        people = []
        for person in msg.people:
            # make the person position relative to the non rotating robot
            X = (person.position.x - t.transform.translation.x)*self.density
            Y = (person.position.y - t.transform.translation.y)*self.density
            if abs(X) < self.center[0] or abs(Y) < self.center[1]:
                X = np.floor(X + self.center[0])
                Y = np.floor(Y + self.center[1])
                # rotate LUT result for specific velocity
                vel = (person.velocity.x*person.velocity.x +
                       person.velocity.y*person.velocity.y)**0.5

                social_zone = rotate(
                    LUT(vel), person.position.z, reshape=True)

                (width, height) = np.shape(social_zone)
                social_map[X-width:X+width, Y-height:Y +
                           height] = np.maximum(social_map[X-width:X+width, Y-height:Y +
                                                           height], social_zone)


def main(args=None):
    rclpy.init(args=args)

    social_map_generator = SocialMapGenerator()
    rclpy.spin(social_map_generator)
    social_map_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
