"""Functions for modeling ROSBot"""

from math import cos, sin
import numpy as np
#import me416_utilities as mu
#import rclpy
#from rclpy.node import Node

class KeysToVelocities():
    """ Translate (sequences) of key presses into velocities for the robot """
    def __init__(self, speed_linear=0.0, speed_angular=0.0, speed_delta=0.2, text_description=''):
        """Initializes variables"""
        self.speed_linear = speed_linear
        self.speed_angular = speed_angular
        self.speed_delta = speed_delta
        self.text_description = text_description

    def update_speeds(self, key):
        """Adjusts speeds based on key pressed"""
        # Makes it case insensitive
        key = key.lower()

        # Changes linear and angular speeds
        if key == 'w':
            self.speed_linear = self.speed_linear + self.speed_delta
            self.text_description = "Linear speed increased"
        elif key == 's':
            self.speed_linear = self.speed_linear - self.speed_delta
            self.text_description = "Linear speed decreased"
        elif key == 'a':
            self.speed_angular = self.speed_angular + self.speed_delta
            self.text_description = "Angular speed increased"
        elif key == 'd':
            self.speed_angular = self.speed_angular - self.speed_delta
            self.text_description = "Angular speed decreased"
        elif key == 'z':
            self.speed_linear = 0.0
            self.text_description = "Linear speed set to 0"
        elif key == 'c':
            self.speed_angular = 0.0
            self.text_description = "Angular speed set to 0"
        elif key == 'x':
            self.speed_linear = 0.0
            self.speed_angular = 0.0
            self.text_description = "Linear and angular speeds set to 0"
        else:
            self.text_description = "Not a command"

        # Thresholds speeds between -0.1 and 0.1
        if self.speed_linear>1:
            self.speed_linear=1.0
        elif self.speed_linear<-1:
            self.speed_linear=-1.0
        if self.speed_angular>1:
            self.speed_angular=1.0
        elif self.speed_angular<-1:
            self.speed_angular=-1.0
        return self#(self.text_description,self.speed_linear,self.speed_angular)

def model_parameters():
    """Returns two constant model parameters"""
    param_k = 1.0
    param_d = 0.5
    return param_k, param_d

def twist_to_speeds(speed_linear, speed_angular):
    """Returns the motor speeds of the left and right wheels based on the linear and
    angular velocities of the robot given as inputs"""
    right=speed_linear+speed_angular*speed_linear
    left=speed_linear-speed_angular*speed_linear
    max_velocity=max(abs(left),abs(right))
    if max_velocity>1:
        right=right/abs(right)
        left=left/abs(left)
    return left, right
    #Formulas for left and right speeds determined from these conditions:
    # 1) If angular speed is >0, then right-left=positive # =angular_speed
    #   and if angular speed < 0, then left-right=positive # =angular_speed
    #   (can set positive # equal to the angular speed)
    # 2) The average of the left and right motor speeds have to have the
    #same sign as the linear speed,
    #   (can say: linear_speed = (left + right)/2 = mean(left,right)).
    #By solving this system of equations, eft and right values are obtained.

def system_matrix(theta):
    """Returns a numpy array with the A(theta) matrix for a differential drive robot"""
    #theta (type float ): angle of the rotation from the body-fixed frame on the
    #robot and the world reference frame
    k_param,d_param=model_parameters()
    A=np.array([[cos(theta),cos(theta)],[sin(theta), sin(theta)],[-1/d_param,1/d_param]],dtype=float)
    return A*k_param/2


#def system_field(z, u):
    #"""Computes the field at a given state for the dynamical model"""
    #return dot_z


#def euler_step(z, u, stepSize):
    #"""Integrates the dynamical model for one time step using Euler's method"""
    #return z_next
