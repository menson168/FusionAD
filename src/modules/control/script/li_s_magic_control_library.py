import scipy.linalg as la
import matplotlib.pyplot as plt
import math
import numpy as np
import sys

class control_param:
    def __init__(self, time_step, wheel_base, max_steering_angle_degree):

class vehicle_state:
    def __init__(self, x, y, yaw, velocity):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.velocity = velocity

class lqr_lateral_control:
    def __init__(self, lqr_gain, control_param):
        self.lqr_gain = lqr_gain
        self.control_param = control_param

    def lqr_control():
        
