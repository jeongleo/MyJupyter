import math

class Particle:
    fric_k_e = 0.2
    fric_s_e = 0.4
    gravity_acc = 9.81
    
    def __init__(self, weight, position, velocity):
        self.weight = weight
        self.position = position
        self.velocity = velocity
        
        self.fric_k = Particle.fric_k_e * Particle.gravity_acc * self.weight
        self.fric_s_max = Particle.fric_s_e * Particle.gravity_acc * self.weight
    
    def force(self, force, dt):
        
        if not self.isMove(): # 정지상태일때
            if abs(force) > self.fric_s_max: # 최대정지마찰력보다 크면
                force = math.copysign(abs(force) - self.fric_k, force)
            else:
                force = 0.0
        else: # 운동상태일때
            force = math.copysign(abs(force) - self.fric_k, force)
        
        acc = force / self.weight
        self.position += self.velocity*dt + acc*(dt**2)*0.5
        self.velocity += acc*dt
        
        self.isMove()
        
        return (acc, self.velocity, self.position)
    
    def isMove(self):
        if abs(self.velocity) < 0.000001:
            self.velocity = 0
            return 0
        else:
            return 1