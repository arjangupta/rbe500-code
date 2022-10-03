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

    def calculate_inverse_kinematics(self, orientation, position):
        """
        This function executes the inverse kinematics for Problem 3.5
        of the RBE 500 main textbook.
        """
        print(f"\nReceived orientation: {orientation} and position: {position}, forming homoegenous transformation from these values.")

        # Use the quaternion to convert to a rotation matrix, and append on the position vector to the right
        # side. This will give us the transformation matrix that gets us to the wrist center (T_0_3 matrix where
        # zero is in the super script and 3 is in the subscript).
        # The quaternion -> rotation matrix is given in the following Wikipedia link:
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Rotation_matrices. We
        # choose the homoegenous matrix expression because we are forming a homogeneous transformation. We will
        # delcare variables q0, q1, q2, q3 just like the wikipedia page so that it is easy to check our work.
        q0 = orientation.w
        q1 = orientation.x
        q2 = orientation.y
        q3 = orientation.z
        # Declare x_c, y_c, z_c to denote the position of the wrist center, just like the way it
        # is described in our main textbook.
        x_c = position.x
        y_c = position.y
        z_c = position.z
        # T_0_3 indicates 0 in the super script, 3 in the subscript. So, it is the homogeneous 
        # transformation that gives the position and orientation of frame 3 with respect to 0. 
        T_0_3 = np.array(
            [[(q0*q0 + q1*q1 - q2*q2 - q3*q3), 2*(q1*q2 - q0*q3),               2*(q0*q2 + q1*q3),               x_c],
             [2*(q1*q2 + q0*q3),               (q0*q0 - q1*q1 + q2*q2 - q3*q3), 2*(q2*q3 - q0*q1),               y_c],
             [2*(q1*q3 - q0*q2),               2*(q0*q1 + q2*q3),               (q0*q0 - q1*q1 - q2*q2 + q3*q3), z_c],
             [0,                               0,                               0,                               1]])
        print(f"The wrist center T matrix is given by\n{T_0_3}\n")

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
        print(f"D is {D}")
        print(f"sqrt of 1 - D^2 is {1*math.sqrt(1 - D**2)} and {-1*math.sqrt(1 - D**2)}")
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
        self.calculate_inverse_kinematics(msg.orientation, msg.position)

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