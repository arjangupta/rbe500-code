import rclpy
import rclpy.node
import std_msgs.msg
import geometry_msgs.msg
import math
import numpy as np

# This class is our ROS Node which will perform both forward and
# inverse kinematics based on the subscription topic on which we
# send the data.
class BidirectionalKinematics(rclpy.node.Node):
    def __init__(self) -> None:
        # Give a name to this node
        super().__init__('BidirectionalKinematics')
        # Subscribe on the forward kinematics topic, have it expect
        # an array of floats as its incoming data, execute the
        # forward_kinematics_callback function when data is received,
        # allow a queue of 15 messages
        self.subscription = self.create_subscription(
            std_msgs.msg.Float32MultiArray,
            'forward_kinematics_topic',
            self.subscription_callback,
            15
        )
        # Subscribe on the inverse kinematics topic, have it expect
        # an Pose message its incoming data, execute the
        # inverse_kinematics_callback function when data is received,
        # allow a queue of 15 messages
        self.subscription = self.create_subscription(
            geometry_msgs.msg.Pose,
            'inverse_kinematics_topic',
            self.subscription_callback,
            15
        )

    def calculate_inverse_kinematics(self, orientation, position):
        pass
    
    def calculate_forward_kinematics(self, theta1, theta2, theta3):
        # This function calculates the forward kinematics equations
        # for Problem 3.5 of the RBE 500 Main Textbook
        
        # First, assign values for a2, a3, d1
        a2 = 1
        a3 = 1
        d1 = 1

        # Now calculate sines and cosines using the revolute joint variables
        c1 = math.cos(theta1)
        s1 = math.sin(theta1)
        c2 = math.cos(theta2)
        s2 = math.sin(theta2)
        c3 = math.cos(theta3)
        s3 = math.sin(theta3)

        # Using the MATLAB symbolic calculation of the T matrix in the HW2
        # assignment, write the T matrix 
        T_matrix = np.array([[c1*c2*c3-c1*s2*s3, -c1*c2*s3-c1*c3*s2, -s1, a2*c1*c2-a3*c1*s2*s3+a3*c1*c2*c3],
                             [c2*c3*s1-s1*s2*s3, -c2*s1*s3-c3*s1*s2, c1, a2*c2*s1-a3*s1*s2*s3+a3*c2*c3*s1],
                             [-c2*s3-c3*s2, s2*s3-c2*c3, 0, d1-a2*s2-a3*c2*s3-a3*c3*s2],
                             [0, 0, 0, 1]])
        print(f'\nThe end effector pose for the given values {theta1}, {theta2}, {theta3} is represented by the following T_matrix:\n{T_matrix}\n')

    def forward_kinematics_callback(self, msg):
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
    # Instantiate the BidirectionalKinematics class
    bidirectional_kinematics = BidirectionalKinematics()
    # Spin the client
    rclpy.spin(bidirectional_kinematics)

    # Upon Ctrl+C, clean up
    bidirectional_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()