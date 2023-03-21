#!/usr/bin/env python3
"""
Repeater that receives strings messages from the 'chatter'
 topic, modifies them, and then sends them on the 'chatter_repeated' topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerAccumulator(Node):
    '''
    Node that uses a timer to publish on /chatter
    '''
    def __init__(self):
        '''
        Setup subscriber and publisher
        '''
        super().__init__('ListenerAccumulator')
        self.subscriber = self.create_subscription(String, 'chatter',
                                                   self.msg_callback, 10)
        self.publisher = self.create_publisher(String, 'chatter_repeated', 10)
        '''
        Create timer to trigger publish of concatenated messages
        '''
        timer_period=3.0
        self.timer = self.create_timer(timer_period, self.timer_callback)#publish based on timer_period
        self.accumulated_messages = "" #had to be initially empty

    def msg_callback(self, msg):
        """
        Callback to receive a message and repeat it with some modifications
        """
        # Append the received message to the accumulated_messages string
        self.accumulated_messages += msg.data + " " #have message appended
        # Log the received message this helped with debugging
        #self.get_logger().info(f"I heard: '{msg.data}'")

    def timer_callback(self):
        """
        Callback for the timer to publish the accumulated messages
        """
        # Only publish if its not " ""
        if self.accumulated_messages:
            # Publish the accumulated messages
            accumulated_msg = String()
            accumulated_msg.data = self.accumulated_messages
            self.publisher.publish(accumulated_msg)

            # Log the published message
            self.get_logger().info(f"I published: '{accumulated_msg.data}'")

            # Reset the accumulated messages string
            self.accumulated_messages = ""


def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    listener_accumulator = ListenerAccumulator()
    rclpy.spin(listener_accumulator)

    # node cleanup
    listener_accumulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
