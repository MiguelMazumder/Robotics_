"""Functions for modeling ROSBot"""

from math import cos, sin
import numpy as np

class KeysToVelocities():   # pylint: disable=too-few-public-methods
    """Translate (sequences) of key presses into velocities for the robot"""
    def __init__(self):
        self.speed_linear=0.0
        self.speed_angular=0.0
        self.SPEED_DELTA=0.2

    def update_speeds(self, key):
        """Uses key input to update the linear and angular speeds"""
        if key.lower() == 'w':
            self.speed_linear = self.speed_linear+self.SPEED_DELTA
            text_description = 'Increase linear speed'
        elif key.lower() == 's':
            self.speed_linear = self.speed_linear-self.SPEED_DELTA
            text_description = 'Decrease linear speed'
        elif key.lower() == 'a':
            self.speed_angular = self.speed_angular+self.SPEED_DELTA
            text_description = 'Increase angular speed'
        elif key.lower() == 'd':
            self.speed_angular = self.speed_angular-self.SPEED_DELTA
            text_description = 'Decrease angular speed'
        elif key.lower() == 'z':
            self.speed_linear = 0.0
            text_description = 'Linear speed set to 0'
        elif key.lower() == 'c':
            self.speed_angular = 0.0
            text_description = 'Angular speed set to 0'
        elif key.lower() == 'x':
            self.speed_linear = 0.0
            self.speed_angular = 0.0
            text_description = 'Linear and angular speed set to 0'
        else:
            text_description = 'Invalid command'

        self.speed_linear = max(self.speed_linear, -1.0)
        self.speed_linear = min(self.speed_linear, 1.0)
        self.speed_angular = max(self.speed_angular, -1.0)
        self.speed_angular = min(self.speed_angular, 1.0)
        if abs(self.speed_linear) < 0.05:
            self.speed_linear = 0.0
        if abs(self.speed_angular) < 0.05:
            self.speed_angular = 0.0
        return self.speed_linear, self.speed_angular, text_description

def twist_to_speeds(speed_linear, speed_angular):
    """Takes linear and angular speeds and calculates the motor speeds needed"""
    left = 0.0
    right = 0.0
    param_k, param_d = model_parameters()
    left = (speed_linear - param_d*speed_angular)/param_k
    right = (speed_linear + param_d*speed_angular)/param_k
    left = max(left, -1.0)
    left = min(left, 1.0)
    right = max(right, -1.0)
    right = min(right, 1.0)
    return left, right


def model_parameters():
    """Returns two constant model parameters"""
    param_k = 1.0
    param_d = 0.5
    return param_k, param_d

def system_matrix(theta):
    """Returns a numpy array with the A(theta) matrix for a differential drive robot"""
    param_k, param_d = model_parameters()
    a_theta = (param_k/2.)*np.array([
        [cos(theta),cos(theta)],
        [sin(theta),sin(theta)],
        [-1./param_d,1./param_d]
        ])
    return a_theta


#def system_field(z, u):
#    """Computes the field at a given state for the dynamical model"""
#    return dot_z


def euler_step(z_current, u_input, step_size):
    """Integrates the dynamical model for one time step using Euler's method"""
    z_next = z_current + step_size*(system_matrix(z_current[2])@u_input)
    return z_next
