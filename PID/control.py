
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from particle import Particle
from pid import PID

acc = []
pos = []
vel = []

# particle init condition
weight = 10.0
position = 10.0
velocity = 0.0

# time condition
init_time = 0.0
final_time = 100
dt = 0.01

# pid param
target = 0.0
kp = 100
ki = 0
kd = 100

class Control:
    def __init__(self):
        self.point = None
        self.timestamp = []
        
        self.pid = None
    
    def set_point(self, weight, position, velocity):
        self.point = Particle(weight, position, velocity)
    
    def set_PID(self, p, i, d, que_len):
        self.pid = PID(que_len)
        self.pid.set_param(p, i, d)
    
    def set_timestamp(self, init, final, dt):
        self.timestamp = np.arange(init_time, final_time, dt)
    
    def start(self, target, dt):
        if type(target) != list:
            target = [target]
        sep = len(self.timestamp)/len(target)
        
        acc = []
        pos = []
        vel = []
        now_target = target[0]
        for i, a in enumerate(self.timestamp):
            for count, x in enumerate(target):
                if i > sep*count:
                    now_target = x

            a, v, p = self.point.force(self.pid.calc(self.point.position, now_target, dt), dt)
            acc.append(a)
            pos.append(p)
            vel.append(v)
        
        return pd.DataFrame({"acc":acc, "vel":vel, "pos":pos}, index=self.timestamp)

if __name__ == '__main__':
    ctl = Control()
    ctl.set_point(10, 1, 0)
    ctl.set_PID(1, 0, 0, 100)
    ctl.set_timestamp(0, 30, 0.01)
    ctl.start([0], dt)
    