from controller.PID import PID
import numpy as np
import matplotlib.pyplot as plt

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

    def sim_accel(self, velocity, accel): #deprecated
        return (accel) - (1.0/(2.0 * self.mass)) * self.rho * self.area * self.drag * (velocity ** 2)
        #return accel

    def getVelocity(self, accel, v0, time_step):
        return accel * time_step + v0

    def torqueToAccel(self, velocity, torque):
        rolling_resistance = 6.0430
        drive_ratio = 3.5
        tire_radius = 0.202
        acc_new = 9.61
        return min((torque * drive_ratio - rolling_resistance - (1.0/(2.0 * self.mass)) * self.rho * self.area * self.drag * (velocity ** 2))/tire_radius/self.mass, acc_new)

    def accelToTorque(self, velocity, accel):
        rolling_resistance = 6.0430
        drive_ratio = 3.5
        tire_radius = 0.202
        max_torque = 230
        return min(((accel * tire_radius * self.mass) + rolling_resistance + (1.0/(2.0 * self.mass)) * self.rho * self.area * self.drag * (velocity ** 2))/drive_ratio, max_torque)
    
    def simulate(self, duration, commands, pid):
    #duration: time to run simulation
    #set of commands to pass through simulator: [time, target]

        currCommand = 0 #index for current command
        v = [0] #list to track previous velocities for model
        a = [0]

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

            targetList.append(pid.getTarget())
            v.append(self.getVelocity(a[-1], v[-1], time_step_size))

            if time_list[i] % pid.sample_rate:
                signal = pid.compute(a[-1]) #returns acceleration
                outputList.append(self.accelToTorque(v[-1], signal))

            #a.append(self.sim_accel(v[-1], outputList[-1]))
            a.append(signal)

        return time_list, outputList, a, targetList, v
    
if( __name__) == ("__main__"):
    pid = PID(1, 0.1, 0.1, 12, 0.1) 
    car = model(300, 0.24, 1.225, 5)
    commandList = [[0, 5], [1,4], [2,-3], [3, 10]]

    timeList, outputList, a, targetList, v = car.simulate(5, commandList, pid)
    #print(timeList)
    #print(outputList)
    #print(a)
    #print(targetList)
    fig, (ax1,ax2, ax3) = plt.subplots(3, sharex = True, constrained_layout = True)

    ax1.plot(timeList, targetList, label = "target")
    ax1.plot(timeList, a, label = "accel")
    ax1.legend()
    ax1.set_ylabel("acceleration")
    ax1.set_title("target and acceleration")

    ax2.plot(timeList, v, label = "velocity")
    ax2.set_ylabel("velocity")
    ax2.set_title("velocity over time")

    ax3.plot(timeList, outputList)
    ax3.set_ylabel("torque")
    ax3.set_title("torque over time")
    plt.show(block = True)