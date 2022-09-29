import rclpy
import rclpy.node
import std_msgs.msg
import math
import numpy as np

class ForwardKinematics(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__('ForwardKinematics')
        self.subscription = self.create_subscription(
            std_msgs.msg.Float32MultiArray,
            'forward_kinematics_topic',
            self.subscription_callback,
            15
        )
    
    def calculate_forward_kinematics(self, theta1, theta2, theta3):
        # This function calculates the forward kinematics equations
        # for Problem 3.5 of the RBE 500 Main Textbook
        T_matrix = np.array([[0]*4]*4)
        print(T_matrix)
    
    def subscription_callback(self, msg):
        # Check input
        if len(msg.data) != 3:
            print("\nYou have not provided 3 angles to this subscriber. No conversion has been done.")
            return
        # Extract data from message
        theta1 = msg.data[0]
        theta2 = msg.data[1]
        theta3 = msg.data[2]
        self.calculate_forward_kinematics(theta1, theta2, theta3)

def main():
    # Initialize the ros2 client library
    rclpy.init()
    # Instantiate the ForwardKinematics class
    euler_to_quat = ForwardKinematics()
    # Spin the client
    rclpy.spin(euler_to_quat)

    # Upon Ctrl+C, clean up
    euler_to_quat.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()