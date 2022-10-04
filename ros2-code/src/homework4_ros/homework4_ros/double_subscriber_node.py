import rclpy
import rclpy.node
import std_msgs.msg
import geometry_msgs.msg
import math
import numpy as np

class BidirectionalKinematics(rclpy.node.Node):
    """
    This class is our ROS Node which will perform both forward and
    inverse kinematics based on the subscription topic on which we
    send the data.
    """
    def __init__(self) -> None:
        # Give a name to this node
        super().__init__('BidirectionalKinematics')
        # Subscribe on the forward kinematics topic, have it expect
        # an array of floats as its incoming data, execute the
        # forward_kinematics_callback member function when data
        #  is received, allow a queue of 15 messages
        self.subscription = self.create_subscription(
            std_msgs.msg.Float32MultiArray,
            'forward_kinematics_topic',
            self.forward_kinematics_callback,
            15
        )
        # Subscribe on the inverse kinematics topic, have it expect
        # an Pose message its incoming data, execute the
        # inverse_kinematics_callback member function when data
        # is received, allow a queue of 15 messages
        self.subscription = self.create_subscription(
            geometry_msgs.msg.Pose,
            'inverse_kinematics_topic',
            self.inverse_kinematics_callback,
            15
        )

    """
    The robot Problem 3.5 is a three-link articulated robot with only
    three constants in its set up: d1, a2, and a3. Let us defined these
    as class variables for this node 
    """
    d1 = 1
    a2 = 1
    a3 = 1

    def calculate_forward_kinematics(self, theta1, theta2, theta3):
        """
        This function calculates the forward kinematics equations
        for Problem 3.5 of the RBE 500 Main Textbook.
        """
        # First, assign values for a2, a3, d1
        a2 = self.a2
        a3 = self.a3
        d1 = self.d1

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
        """
        This function is executed when the forward kinematics
        topic receives data.
        """
        # Check input
        if len(msg.data) != 3:
            print("\nYou have not provided 3 angles to this subscriber. No conversion has been done.")
            return
        # Extract data from message
        theta1 = msg.data[0]
        theta2 = msg.data[1]
        theta3 = msg.data[2]
        self.calculate_forward_kinematics(theta1, theta2, theta3)

    def calculate_inverse_position(self, position):
        """
        This function executes the inverse position for Problem 3.5
        of the RBE 500 main textbook.
        """
        print(f"\nInverse position calculator received position: {position}")
        # Declare x_c, y_c, z_c to denote the position of the wrist center, just like the way it
        # is described in our main textbook.
        x_c = position.x
        y_c = position.y
        z_c = position.z
        # Compute theta1, theta2, and theta3 as given in our book.
        # For theta1, we use equations 5.18 and 5.19 of our textbook
        theta1_option1 = math.atan2(y_c, x_c)
        theta1_option2 = math.pi + theta1_option1
        print(f"For theta1, the two options are {theta1_option1} and {theta1_option2}")
        # Before computing theta2 and theta3, compute r and s, which are
        # defined right before equation 5.25 of our textbook.
        r = x_c**2 + y_c**2
        s = z_c - self.d1
        # Use equation 5.25 of textbook to find theta3
        D = (r**2 + s**2 - (self.a2)**2 - (self.a3)**2)/(2*self.a2*self.a3)
        theta3_option1 = math.atan2(math.sqrt(1 - D**2), D)
        theta3_option2 = math.atan2(-1*math.sqrt(1 - D**2), D)
        # Use equation 5.26 of textbook to find theta2
        theta2_option1 = math.atan2(s, r) - math.atan2(self.a3*math.sin(theta3_option1), self.a2 + self.a3*math.cos(theta3_option1))
        theta2_option2 = math.atan2(s, r) - math.atan2(self.a3*math.sin(theta3_option2), self.a2 + self.a3*math.cos(theta3_option2))
        print(f"For theta2, the two options are {theta2_option1} and {theta2_option2}")
        print(f"For theta3, the two options are {theta3_option1} and {theta3_option2}")
    
    def inverse_kinematics_callback(self, msg):
        """
        This function is executed when the inverse kinematics
        topic receives data.
        """
        # Only use the position portion of the Pose message because
        # we only have three joints. We would use the orientation
        # portion as well if we had 3 more joints. 
        self.calculate_inverse_position(msg.position)

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