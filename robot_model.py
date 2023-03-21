"""Functions for modeling ROSBot"""

from math import cos, sin
import numpy as np


def model_parameters():
    """Returns two constant model parameters"""
    param_k = 1.0
    param_d = 0.5
    return param_k, param_d

def twist_to_speeds(speed_linear, speed_angular):
    """Returns the motor speeds of the left and right wheels based on the linear and
    angular velocities of the robot given as inputs"""
    if not -1.0<=speed_linear<= 1.0 or not -1.0<=speed_angular <= 1.0 or (speed_angular==0 and speed_linear!=0):
        # If either the linear or angular speeds are outside the range of [-1.0,1.0],
        # or if the robot is commanded to turn without moving forward,
        # then the robot should not move.
        left = 0.0
        right = 0.0
    elif abs(speed_angular) < 0.1:
        # If the absolute value of the angular velocity is small enough to
        #be negligible (less than 0.1),
        # then the left motor should move at roughly the same speed as the
        #right motor which is given by the inputted linear velocity.
        left = speed_linear
        right = speed_linear
    else:
        # Otherwise, compute left right motor speeds using the differential drive model.
        omega = speed_linear / speed_angular
        left = speed_linear - (omega / 2.0)
        right = speed_linear + (omega / 2.0)

    # Return the computed left and right motor speeds as a tuple.
    return (left, right)
    #Formulas for left and right speeds determined from these conditions:
    # 1) If angular speed is >0, then right-left=positive # =angular_speed
    #   and if angular speed < 0, then left-right=positive # =angular_speed
    #   (can set positive # equal to the angular speed)
    # 2) The average of the left and right motor speeds have to have the
    #same sign as the linear speed,
    #   (can say: linear_speed = (left + right)/2 = mean(left,right)).
    #By solving this system of equations, eft and right values are obtained.
    
#def system_matrix(theta):
    #"""Returns a numpy array with the A(theta) matrix for a differential drive robot"""
    #return a_matrix


#def system_field(z, u):
    #"""Computes the field at a given state for the dynamical model"""
    #return dot_z


#def euler_step(z, u, stepSize):
    #"""Integrates the dynamical model for one time step using Euler's method"""
    #return z_next
