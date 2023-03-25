#!/usr/bin/env python
"""Simple script that waits for the key 'q' and then terminates"""

import me416_utilities as mu
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import robot_model


class KeyTerminate(Node):
    '''
    Simple Node class that uses a timer to check for key presses
    '''
    def __init__(self):
        super().__init__('key_terminate')
        self.timer = self.create_timer(
            1. / 50,  # Check keys at 50Hz
            self.timer_callback)
        # Boilerplate to get function to read keyboard
        self.getch = mu._Getch()
        self.is_running = True
        self.publisher = self.create_publisher(Twist, 'robot_twist', 10)

    def timer_callback(self):
        '''
        Check if the given key was pressed
        '''
        key = self.getch()
        test=robot_model.KeysToVelocities()
        if key == 'q':
            self.get_logger().info(f'Shutdown initiated by {self.get_name()}')
            self.destroy_node()
            self.is_running = False
        else:
            updated=test.update_speeds(key)
        msg = Twist()
        msg.linear.x=updated.speed_linear
        msg.linear.y=0.0
        msg.linear.z=0.0
        msg.angular.x=0.0
        msg.angular.y=0.0
        msg.angular.z=updated.speed_angular
        self.publisher.publish(msg)
        print(msg.linear.x)
        print(msg.angular.z)


def main(args=None):
    """Function to setup node and loop"""
    print('\tPress "q" to quit.\n')
    rclpy.init(args=args)
    node = KeyTerminate()
    while node.is_running:
        rclpy.spin_once(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
