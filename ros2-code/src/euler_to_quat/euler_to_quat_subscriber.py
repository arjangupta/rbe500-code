import rclpy
import std_msgs # (TODO: remove this comment) for using Float32MultiArray.msg

def main():
    # Initialize the ros2 client library
    rclpy.init()
    print("Hi, this is the euler_to_quat package!")

if __name__ == '__main__':
    main()