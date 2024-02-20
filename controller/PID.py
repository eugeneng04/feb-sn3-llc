#import rospy

class PID(object):
    def __init__(self, Kp, Ki, Kd, sample_rate = 0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.sample_rate = sample_rate
        self.output = 0
        self.target = 0
        self.prev_error = 0
        self.time = 0
        self.prev_time = 0
        self.integral = 0
        self.max_output = 1
    
    def getGains(self):
        return (self.Kp, self.Ki, self.Kd)
    
    def setGains(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
    
    def setTarget(self, target):
        self.integral = 0
        print(f"setting target from {self.target} to {target}")
        self.target = target

    def getTarget(self):
        return self.target
    
    def compute(self, current):
        error = self.target - current
        derivative = (error - self.prev_error)/self.sample_rate

        self.integral += error

        output = (error * self.Kp) + (self.integral * self.Ki) + (derivative * self.Kd)

        if self.output > self.max_output:
            self.output = self.max_output
        elif self.output < -self.max_output:
            self.output = -self.max_output
        print(output)
        self.prev_error = error

        return output
    
    