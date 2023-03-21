""" Example of how to set attributes in ROS messages """
from geometry_msgs.msg import Twist

def twist_fill():
    '''
    Fill a twist message with non-zero values
    '''
    msg = Twist()
    msg.linear.x=2.0
    msg.linear.y=3.0
    msg.linear.z=5.0
    msg.angular.x=7.0
    msg.angular.y=6.0
    msg.angular.z=9.0
    return msg
