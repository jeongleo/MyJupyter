from collections import deque

class PID:
    def __init__(self, que=100):
        self.k_p = 1.0
        self.k_i = 0.0
        self.k_d = 0.0
        
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0
        
        self.que_len = que
        self.i_que = deque([0]*self.que_len)
        
        self.p_error = None
        
        self.max_limit = 200
        self.min_limit = -200
    
    def calc(self, now, target, dt):
        error = target - now
        
        if self.p_error == None:
            self.p_error = error
        
        self.p = self.k_p * error
        self.d = self.k_d * (error - self.p_error)/dt
#         self.i += self.k_i * error * dt
        
        self.i_que.popleft()
        self.i_que.append(error * dt)
        self.i = self.k_i * sum(self.i_que)
        
        self.p_error = error
        
        return self.strain(self.p + self.i + self.d)
    
    def set_param(self, p, i, d):
        self.k_p = p
        self.k_i = i
        self.k_d = d
    
    def strain(self, value):
        if value > self.max_limit:
            return self.max_limit
        elif value < self.min_limit:
            return self.min_limit
        else:
            return value