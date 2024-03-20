import rclpy
from rclpy.node import Node

import numpy as np
from geometry_msgs.msg import Pose

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.last_pose = np.array([0, 0, 0, 0, 0, 0, 0])
        self.subscription = self.create_subscription(
            Pose,
            'mocap4r2_optitrack_driver',
            self.listener_callback,
            10)  #queue list
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.last_pose = msg.data

    def signal(self):
        return self.last_pose
    
    def velocity(self):
        velocity = self.vel_filter()
        return velocity
    
    def vel_filter(self):
        return None

    def main(args=None):
        rclpy.init(args=args)

        minimal_subscriber = MinimalSubscriber()

        rclpy.spin(minimal_subscriber)

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

# rclpy.Node.Subscriber('/mocap4r2_optitrack_driver/RigidBody/pose', Pose, some_callback)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()