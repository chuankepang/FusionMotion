import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from cust_msgs.msg import Stampfloat32array  # Assuming cust_msgs package is installed and available
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Define QoS profile similar to the C++ code
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        qos.deadline = rclpy.duration.Duration(nanoseconds=1_000_000)  # 1ms

        # Create publishers
        self.pose_pub = self.create_publisher(Float32MultiArray, 'sent_pose', qos)
        self.force_pub = self.create_publisher(Stampfloat32array, 'force_data', qos)
        self.joint_pub = self.create_publisher(Float32MultiArray, 'sent_joints', qos)

        # Timer to publish data every 0.1 seconds
        self.timer = self.create_timer(0.1, self.publish_data)

        self.get_logger().info('Simple publisher node started. Publishing to sent_pose, force_data, and sent_joints.')

    def publish_data(self):
        # Publish to sent_pose: example 6 floats (position + orientation)
        pose_msg = Float32MultiArray()
        pose_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Dummy data
        self.pose_pub.publish(pose_msg)
        self.get_logger().info('Published to sent_pose')

        # Publish to force_data: custom message with stamp and 6 floats
        force_msg = Stampfloat32array()
        #force_msg.stamp = self.get_clock().now().to_msg()  # Current timestamp
        force_msg.data = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # Dummy data
        self.force_pub.publish(force_msg)
        self.get_logger().info('Published to force_data')

        # Publish to sent_joints: example 7 floats for 7 DoF
        joint_msg = Float32MultiArray()
        joint_msg.data = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]  # Dummy data
        self.joint_pub.publish(joint_msg)
        self.get_logger().info('Published to sent_joints')

def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()