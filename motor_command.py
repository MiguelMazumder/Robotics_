#!/usr/bin/env python3
"""
Repeater that receives strings messages from the 'chatter'
 topic, modifies them, and then sends them on the 'chatter_repeated' topic
"""
import robot_model
import rclpy
from rclpy.node import Node
from me416_utilities import MotorSpeedLeft, MotorSpeedRight
from geometry_msgs.msg import Twist
from me416_msgs.msg import MotorSpeedsStamped

motor_left = MotorSpeedLeft()
motor_right = MotorSpeedRight()
    
class MotorCommand(Node):
    def __init__(self):
        super().__init__('MotorCommand')#line I added
        self.subscriber = self.create_subscription(Twist, 'robot_twist', self.msg_callback, 10)
        self.publisher = self.create_publisher(MotorSpeedsStamped, 'motor_speeds', 10)

    def msg_callback(self, msg):
        left,right = robot_model.twist_to_speeds(msg.linear.x,msg.angular.z)
        motor_left.set_speed(left)
        motor_right.set_speed(right)

        msg = MotorSpeedsStamped()
        msg.header.stamp= self.get_clock().now().to_msg()
        msg.left = float(motor_left)
        msg.right = float(motor_right)
        self.publisher.publish(msg)
              
def main(args=None):
    '''
    Init ROS, launch node, spin, cleanup
    '''
    rclpy.init(args=args)
    motor = MotorCommand()
    rclpy.spin(motor)

    # node cleanup
    motor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
