import rclpy
import rclpy.node
import std_msgs.msg
import math

class EulerToQuat(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__('euler_to_quat')
        self.subscription = self.create_subscription(
            std_msgs.msg.Float32MultiArray,
            'euler_to_quat_topic',
            self.subscription_callback,
            15)
    
    def convert_euler_to_quat(self, psi, theta, phi):
        # As per the conversion formula given on the wikipedia page:
        # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_angles_to_quaternion_conversion
        
        # Calculate the trigonometric ratios
        cos_half_psi   = math.cos(psi/2)
        sin_half_psi   = math.sin(psi/2)
        cos_half_theta = math.cos(theta/2)
        sin_half_theta = math.sin(theta/2)
        cos_half_phi   = math.cos(phi/2)
        sin_half_phi   = math.sin(phi/2)

        # Get the quaternion coordinates
        w = (cos_half_phi * cos_half_theta * cos_half_psi) + (sin_half_phi * sin_half_theta * sin_half_psi)
        x = (sin_half_phi * cos_half_theta * cos_half_psi) - (cos_half_phi * sin_half_theta * sin_half_psi)
        y = (cos_half_phi * sin_half_theta * cos_half_psi) + (sin_half_phi * cos_half_theta * sin_half_psi)
        z = (cos_half_phi * cos_half_theta * sin_half_psi) - (sin_half_phi * sin_half_theta * cos_half_psi)

        return {'w': w, 'x': x, 'y': y, 'z': z}

    def subscription_callback(self, msg):
        # Check input
        if len(msg.data) != 3:
            print("\nYou have not provided 3 angles to this subscriber. No conversion has been done.")
            return
        # Extract data from message
        psi   = msg.data[0]
        theta = msg.data[1]
        phi   = msg.data[2]
        print("\n-----------------------------------------------")
        print(f"Received, psi: {psi}, theta: {theta}, phi: {phi}\n")
        quaternion = self.convert_euler_to_quat(psi, theta, phi)
        print(f"Converted to quaternion, w: {quaternion['w']}, x: {quaternion['x']}, y: {quaternion['y']}, z: {quaternion['z']}")
        print("-----------------------------------------------")


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