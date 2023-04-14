
"""Functions for modeling ROSBot"""

from math import cos, sin
import numpy as np
from rclpy.time import Time

def model_parameters():
    """Returns two constant model parameters"""
    param_k = 1.0
    param_d = 0.5
    return param_k, param_d


def system_matrix(theta):
    """Returns a numpy array with the A(theta) matrix for a differential drive robot"""
    [k,d] = model_parameters()
    theta=float(theta)
    matrix = np.array([[cos(theta),cos(theta)], [sin(theta),sin(theta)], [-1/d,1/d]])
    a_mat = k/2*matrix
    return a_mat


#def system_field(z, u):
    #"""Computes the field at a given state for the dynamical model"""
    #return dot_z


def euler_step(z_current, u_input, step_size):
    """Integrates the dynamical model for one time step using Euler's method"""
    a_z=system_matrix(z_current[-1])#captures just the rotation and not position
    z_next=z_current+step_size*np.matmul(a_z,u_input)
    return z_next

def twist_to_speeds(speed_linear, speed_angular):
    """Calculates motors' speeds"""

    # If the angular speed is 0, then the robot is only moving linearly so both motor speeds should be equal
    if speed_angular == 0:
        # As such, both left and right motors were set to the input linear speed
        left = speed_linear
        right = speed_linear
    # If the angular speed is positive, then the right motor's speed should be greater than the left's
    elif speed_angular > 0:
        # To do this, the left motor was first set to the input linear speed
        left = speed_linear
        # The right motor was then set to linear speed plus the absolute value of the angular speed.
        # This was to make the right motor greater than the left one by an amount relative to the input angular speed.
        # In other words, a larger desired angular speed would result in a larger difference between the motors' speeds
        right = speed_linear + abs(speed_angular)
    # The same logic as for a positive angular speed was applied to a negative angular speed.
    # This just resulted in the left and right values being switched
    elif speed_angular < 0:
        left = speed_linear + abs(speed_angular)
        right = speed_linear

    # Since the motor speeds need to be thresholded between -1.0 and 1.0,
    # this makes it so speeds greater than 1.0 become 1.0 and speeds less than -1.0 become -1.0
    if left > 1:
        left = 1.0
    if left < -1:
        left = -1.0
    if right > 1:
        right = 1.0
    if right < -1:
        right = -1.0

    return left, right

class KeysToVelocities():
    """ Translate (sequences) of key presses into velocities for the robot """
    def __init__(self, speed_linear=0.0, speed_angular=0.0, SPEED_DELTA=0.2):
        """Initialize variables"""
        self.speed_linear = speed_linear
        self.speed_angular = speed_angular
        self.SPEED_DELTA = SPEED_DELTA

    def update_speeds(self, key):
        """Adjust speeds based on key pressed"""
        # Make it case insensitive
        key = key.lower()
        text_description = ''
        
        # Change linear and angular speeds
        if key == 'w':
            self.speed_linear = self.speed_linear + self.SPEED_DELTA
            text_description = "Linear speed increased"
        elif key == 's':
            self.speed_linear = self.speed_linear - self.SPEED_DELTA
            text_description = "Linear speed decreased"
        elif key == 'a':
            self.speed_angular = self.speed_angular + self.SPEED_DELTA
            text_description = "Angular speed increased"
        elif key == 'd':
            self.speed_angular = self.speed_angular - self.SPEED_DELTA
            text_description = "Angular speed decreased"
        elif key == 'z':
            self.speed_linear = 0.0
            text_description = "Linear speed set to 0"
        elif key == 'c':
            self.speed_angular = 0.0
            text_description = "Angular speed set to 0"
        elif key == 'x':
            self.speed_linear = 0.0
            self.speed_angular = 0.0
            text_description = "Linear and angular speeds set to 0"
        else:
            text_description = "Not a valid key input"

        # Threshold speeds between -1.0 and 1.0
        if self.speed_linear < -1.0:
            self.speed_linear = -1.0
        if self.speed_linear > 1.0:
            self.speed_linear = 1.0
        if self.speed_angular < -1.0:
            self.speed_angular = -1.0
        if self.speed_angular > 1.0:
            self.speed_angular = 1.0
        return self.speed_linear, self.speed_angular, text_description
    
class StampedMsgRegister():
    """ Computes the delay, in seconds, between two ROS messages """
    def __init__(self, msg_previous=None):
        """Initialize variables"""
        self.msg_previous = msg_previous
    def replace_and_compute_delay(self, msg):
        """Given new stamped message input, computes delay (sec) between time stamp of
        message and the value in time_previous , andthen replaces the internal copy of
        the previous message with the current message."""
        time_delay=None
        if self.msg_previous is not None:
            time1 = Time.from_msg(self.msg_previous)
            time2 = Time.from_msg(msg)
            time_delay = time2 - time1
            #time_delay = ult.stamp_difference(self.msg_previous,msg)
            msg_previous_copy=self.msg_previous
            self.msg_previous=msg
            #return [time_delay.nanoseconds / 1e9],msg_previous_copy
            return time_delay,msg_previous_copy if self.msg_previous is not None else None
    def previous_stamp(self):
        """returns stamp"""
        stamp=None
        if stamp is not None:
            stamp=self.msg_previous.header.stamp
            return stamp if stamp is not None else None
