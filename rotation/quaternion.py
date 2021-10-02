#

from math import radian, degree, pi, copysign, pow, sqrt
import numpy as np
import scipy as sp


class Quaternion:
    def __init__(self):
        self.q = np.array([1, 0, 0, 0], dtype=np.float64)
        self.rotate_mat = np.zeros([3, 3], dtype=np.float64)
    
    @staticmethod
    def rotation_matrix_quat(q):
        q2 = []
        for i in q:
            q2.append(pow(i, 2))
        
        c = np.array([[q2[0]+q2[1]-q2[2]-q2[3], 2*(q[1]*q[2]+q[0]*q[3]), 2*(q[1]*q[3]-q[0]*q[2])],
                      [2*(q[1]*q[2]-q[0]*q[3]), q2[0]-q2[1]+q2[2]-q2[3], 2*(q[2]*q[3]+q[0]*q[1])],
                      [2*(q[1]*q[3]+q[0]*q[2]), 2*(q[2]*q[3]-q[0]*q[1]), q2[0]-q2[1]-q2[2]+q2[3]]],
                    dtype=np.float64)
        return c
    
    @staticmethod
    def derivative_quaternion(q, w)
    """
    q = [q0, q1, q2, q3] # 쿼터니언
    w = P*i + Q*j + R*k  # 각속도
    """
    q = np.array(q)
    A = np.array([[0, -w[0], -w[1], -w[2]],
                  [w[0], 0, w[2], -w[1]],
                  [w[1], -w[2], 0, w[0]],
                  [w[2], w[1], -w[0], 0]], dtype=np.float64)
    dq = 0.5 * A @ q
    return dq
    
    @staticmethod
    def euler2quaternion(roll, pitch, yaw):
        psi = yaw / 2
        phi = roll / 2
        theta = pitch / 2
        
        sinpsi = np.sin(psi)
        cospsi = np.cos(psi)
        sinphi = np.sin(phi)
        cosphi = np.cos(phi)
        sinthe = np.sin(theta)
        costhe = np.cos(theta)
        
        q1 = cospsi*cosphi*costhe+sinpsi*sinphi*sinthe
        q2 = sinphi*costhe*cospsi-cosphi*sinthe*sinpsi
        q3 = cosphi*sinthe*cospsi+sinphi*costhe*sinpsi
        q4 = cosphi*costhe*sinpsi-sinphi*sinthe*cospsi
        
        return q0, q1, q2, q3
    
    @staticmethod
    def quaternion2euler(q):
        q2 = []
        for i in q:
            q2.append(pow(i, 2))
        
        theta = np.arcsin( -2*(q[1]*q[3]-q[0]*q[2]) )
        phi = np.arccos( (q2[0]-q2[1]-q2[2]+q2[3])/sqrt(1-4*pow((q[1]*q[3]-q[0]*q[2]), 2)) )
        phi = copysign(phi, q[2]*q[3]+q[0]*q[1])
        psi = np.arccos( (q2[0]+q2[1]-q2[2]-q2[3])/sqrt(1-4*pow((q[1]*q[3]-q[0]*q[2]), 2)) )
        psi = copysign(psi, q[1]*q[2]+q[0]*q[3])
        
        return phi, theta, psi  # roll, pitch, yaw


if __name__ == '__main__':
    quat = Quaternion()
    