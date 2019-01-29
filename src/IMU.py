from GY80_IMU import *
import numpy as np
import threading
import sys

class quat(object):
    def __init__(self,real,imag):
        self.r = real
        self.i = np.array(imag)
    
    def __mul__(self, q):
        if type(q) == type(self):
            q1i = np.cross(self.i,q.i) + self.r*q.i + q.r*self.i
            q1r = self.r * q.r - np.dot(self.i,q.i)
            return quat(q1r,q1i)
        else:
            return quat(self.r*q,self.i*q)

    def __div__(self,q):
        if type(q) == type(self):
            return self*q.inv()
        else:
            return quat(self.r/q,self.i/q)

    def __truediv__(self,q):
        if type(q) == type(self):
            return self*q.inv()
        else:
            return quat(self.r/q,self.i/q)

    def __add__(self, q):
        return quat(self.r + q.r, self.i + q.i)

    def __sub__(self, q):
        return quat(self.r - q.r, self.i - q.i)

    def conj(self):
        return quat(self.r,-self.i)

    def norm(self):
        return np.sqrt(self.r**2 + (np.linalg.norm(self.i))**2)

    def inv(self):
        c = self.conj()
        n = self.norm()
        return quat(c.r/n**2, c.i/n**2)
    

class IMU(threading.Thread):
    def __init__(self, sensor):
        threading.Thread.__init__(self)
        self.sens = sensor
        self.g = [0,0,0]
        self.a = [0,0,0]
        self.m = [0,0,0]
        self.q = quat(1,[0,0,0])
        self.dq_old = quat(1,[0,0,0])
        self.T = time.time()
        self.stop = 0
        self.n = 0
        self.start()

    def read(self):
        [self.g,self.a,self.m] = self.sens.read_Devices()

    def atualiza_atitude(self):
        self.read()
        T = time.time()
        dt = T-self.T
        self.T = T
        dq = self.q * quat(0,self.g)/2
        q = self.q + (dq + self.dq_old)*dt/2
        self.dq_old = dq
        n = q.norm()
        self.q = q/n
        self.n = self.n + 1

    def run(self):
        while(not self.stop):
            self.atualiza_atitude()

imu =  IMU(GY80())
while True:
    try:
        print([imu.q.r, imu.q.i])
        print(imu.n)
        time.sleep(0.5)
    except (KeyboardInterrupt, SystemExit):
        imu.stop = 1
        sys.exit()

