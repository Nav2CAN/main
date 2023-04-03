import rclpy
from rclpy import Node
import threading
import numpy as np
from .people_detector import PeopleDetector


def main(args=None):

    rclpy.init(args=args)

  # Start ROS2 node
    dt = 0.05
    multi_person_tracker = MultiPersonTracker(dt)
    # thread = threading.Thread(
    #     target=rclpy.spin, args=(people_detector, ), daemon=True)
    # thread.start()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(multi_person_tracker)
    executor.add_node(multi_person_tracker.people_detector)
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = multi_person_tracker.create_rate(1/dt)

    try:
        while rclpy.ok():
            rate.sleep()
            # Make sure an image has been captured
            people, timestamps = multi_person_tracker.predict()

            # TODO Non-maximum suppression on people
            # DC for data collection run only until a certain amount of people have been detected
            if people_detector.peopleCount == 1000:
                break
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    people_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
