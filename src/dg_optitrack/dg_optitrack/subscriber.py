import rclpy
from rclpy.node import Node
import time
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from mocap4r2_msgs.msg import RigidBodies

from dg_optitrack.kalman_filter import Filter

# RigidBody data specs:
# std_msgs/Header header
# uint32 frame_number
# mocap4r2_msgs/RigidBody[] rigidbodies

# Biped: streaming ID 1049
# rigid_body publishes information type [mocap4r2_msgs/msg/RigidBodies]

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.last_pose = np.array([0, 0, 0, 0, 0, 0, 0])
        self.poses = np.zeros((4, 7))
        self.streaming_id = "1049"
        self.subscription = self.create_subscription(
            RigidBodies,
            'rigid_bodies',
            self.listener_callback,
            10)  #queue list
        self.subscription  # prevent unused variable warning
        self.filter = Filter()

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.rigidbodies)
        for body in msg.rigidbodies:
            if body.rigid_body_name == self.streaming_id:
                position = np.array([body.pose.position.x, body.pose.position.y, body.pose.position.z])
                # quaternion - x y z w
                orientation = np.array([body.pose.orientation.x, body.pose.orientation.y, body.pose.orientation.z, body.pose.orientation.w])
                self.last_pose = np.concatenate((position, orientation), axis=None)
        #self.filter.update_measurement(self.last_pose)

    def signal(self):
        return self.last_pose
    
    # def velocity(self):
    #     estimate = self.filter.estimate()
    #     velocity = estimate[6:]
    #     return velocity


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    #rclpy.spin(minimal_subscriber)

    for i in range(200):
        #time.sleep(0.01)
        rclpy.spin_once(minimal_subscriber, timeout_sec = 0.01)
        print(minimal_subscriber.signal())

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()