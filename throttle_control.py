from controller.PID import PID
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

import rclpy

class throttle_control(object):
    def __init__(self, rho, area, drag,  mass = 308, drive_ratio= 3.5 , rolling_resistance = 6.043, max_torque = 230, tire_radius = 0.202):
        #constants:
        # mass = 308 kg
        # rolling_resistance = 6.043N
        # drive_ratio = 3.5
        # tire_radius = 0.202 m?
        # max_torque = 230 Nm
        self.rho = rho
        self.area = area
        self.drag = drag
        self.mass = mass
        self.drive_ratio = drive_ratio
        self.rolling_resistance = rolling_resistance
        self.max_torque = max_torque
        self.tire_radius = tire_radius

        self.pid = PID(1, 0.1, 0.1, 12, 0.1)

    def getVelocity(self): #pulls velocity from odometry node
        return None 
    
    def getAccel(self): #pulls acceleration from odometry node
        return None
    
    def accelToTorque(self, velocity, accel):
        return min(((accel * self.tire_radius * self.mass) + self.rolling_resistance + (1.0/(2.0 * self.mass)) * self.rho * self.area * self.drag * (velocity ** 2))/self.drive_ratio,self.max_torque)
    
    def drive(self):
        flag = True
        outputs = [] #throttle output

        while flag: #while loop to always set targets
            #subscribe to MPC "/control/throttle"
            #subscribe to something that has a flag to shut loop off
            output = self.pid.compute(target)
            outputs.append(output)
            #publish output
            sleep(0.01)

        