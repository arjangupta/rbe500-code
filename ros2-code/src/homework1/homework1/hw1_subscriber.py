# This is the subscriber portion of Homework 1.
# Prompt: The subscriber will receive these numbers and for each number it will print on the 
# screen “I received [the_number_it_received]. It is an [even/odd] number.” 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# This class performs all the subscribing work. It is a subclass of rclpy.Node
class Homework1Subscriber(Node):

    def __init__(self):
        super().__init__('homework1_subscriber')
        self.subscription = self.create_subscription(Int32,'homework1_topic',self.listener_callback,25)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I received: "%d". It is an even/odd number' % msg.data)

# This function runs the subscriber
def main():
    rclpy.init()

    homework1_subscriber = Homework1Subscriber()

    rclpy.spin(homework1_subscriber)

    homework1_subscriber.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
