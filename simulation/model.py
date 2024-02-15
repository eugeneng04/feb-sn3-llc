from controller.PID import PID
import numpy as np
import matplotlib as plt

class model(object):
    def __init__(self, mass, drag, rho, area):
        #mass: vehicle mass
        #drag: drag coefficient
        #rho: air density
        #area: cross sectional area
        self.mass = mass
        self.drag = drag
        self.rho = rho
        self.area = area

    def sim_accel(self, velocity, accel):
        return (accel) - (1.0/(2.0 * self.mass)) * self.rho * self.area * self.drag * (velocity ** 2)
    
    def getVelocity(self, accel, v0, time_step):
        return accel * time_step + v0
    
    def simulate(self, duration, commands):
        #duration: time to run simulation
        #set of commands to pass through simulator: [time, target]

        currCommand = 0 #index for current command
        v = [0] #list to track previous velocities for model
        a = [0]
        pid = PID(0.1, 0.0, 0.0, 0.1) #create PID object with Kp: 0.1 Ki, Kd = 0, time step  = 0.1

        if commands[0][0] == 0:
            pid.setTarget(commands[0][1]) #set target to first value in the list
            currCommand += 1

        targetList = [pid.getTarget()] #intialize list of targets
        outputList = [pid.compute(a[0])] #initialize output list for accerlation 

        time_step_size = 0.01

        time_list = np.linspace(0, duration, int(np.ceil(duration/time_step_size))) # for plotting

        for i in range(1, len(time_list)):
            if currCommand <= (len(commands) - 1):
                if commands[currCommand][0] <= round(time_list[i], 2):
                    pid.setTarget(commands[currCommand][1])
                    currCommand += 1

            targetList.append(pid.getTarget)
            v.append(self.getVelocity(v[-1], a[-1]))

            if time_list[i] % pid.tS:
                outputList.append(pid.compute(a[-1]))

            a.append(self.sim_accel(v[-1], outputList[-1]))

        return time_list, outputList, v, targetList

