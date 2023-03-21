#!/usr/bin/env python3
"""
Simple demo that publishes to std_msgs/Strings published
to the 'chatter' topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    '''
    Simple Node class that uses a timer to publish on /chatter
    '''
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        #below
        self.colors = ['Pencil', 'Crayon', 'Pen', 'Stylus', 'Marker', 'Highlighter','Quill']
        #above

    def timer_callback(self):
        '''
        Method called by the Timer object
        '''
        msg = String()
        #msg.data = f'Hello World: {self.i}'
        msg.data = f'{self.colors[self.i]}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1
        #below
        if self.i == len(self.colors):
            self.i = 0
        #above


def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    minimal_publisher = Talker()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly, otherwise
    # it will be done automatically
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
