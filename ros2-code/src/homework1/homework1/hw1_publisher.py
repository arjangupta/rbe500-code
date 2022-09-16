# This is the publisher portion of Homework 1.
# Prompt: The publisher will publish increasing integer numbers, e.g. 1, 2, 3, 4, 5, 6, ..., in every one second. 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# This class performs all the publishing work. It is a subclass of rclpy.Node
class Homework1Publisher(Node):

    def __init__(self):
        super().__init__('homework1_publisher')
        self.publisher_ = self.create_publisher(Int32, 'homework1_topic', 25)
        timer_period = 1  # seconds (pubish every 1 second, as per prompt)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publish_number = 1 # as per prompt, we need to start publishing at 1

    def timer_callback(self):
        msg = Int32()
        msg.data = self.publish_number
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.publish_number += 1

# This function runs the publisher
def main():
    rclpy.init()

    homework1_publisher = Homework1Publisher()

    rclpy.spin(homework1_publisher)

    # Destroy the node explicitly
    homework1_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
