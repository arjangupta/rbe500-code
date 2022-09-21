import rclpy
import std_msgs # (TODO: remove this comment) for using Float32MultiArray.msg

class EulerToQuat(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__('euler_to_quat')
        self.subscription = self.create_subscription(
            std_msgs.msg.Float32MultiArray.msg,
            'euler_to_quat_topic',
            self.subscription_callback)

    def subscription_callback(self, msg):
        print("Hi, this is the euler_to_quat package!")


def main():
    # Initialize the ros2 client library
    rclpy.init()
    # Instantiate the EulerToQuat class
    euler_to_quat = EulerToQuat()
    # Spin the client
    rclpy.spin(euler_to_quat)

    # Upon Ctrl+C, clean up
    euler_to_quat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()