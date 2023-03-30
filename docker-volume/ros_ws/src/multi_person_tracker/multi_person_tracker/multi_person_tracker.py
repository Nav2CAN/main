import rclpy
import threading
import numpy as np
from .people_detector import PeopleDetector

def main(args=None):
    
    rclpy.init(args=args)
    people_detector = PeopleDetector(debug=True)  # Start ROS2 node
    thread = threading.Thread(target=rclpy.spin, args=(people_detector, ), daemon=True)
    thread.start()
    

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(people_detector)
    for camera in people_detector.cameras:
        executor.add_node(camera)
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = people_detector.create_rate(10)

    try:
        while rclpy.ok():
            rate.sleep()
             # Make sure an image has been captured
            people, timestamps= people_detector.detect()
            if people_detector.peopleCount == 1000: # DC for data collection run only until a certain amount of people have been detected
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