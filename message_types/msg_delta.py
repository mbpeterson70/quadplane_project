"""
msg_delta
    - messages type for input to the aircraft
    
part of mavsim_python
    - Beard & McLain, PUP, 2012
    - Last update:
        2/27/2020 - RWB
"""
import numpy as np


class MsgDelta:
    def __init__(self,
                 elevator=0.0,
                 aileron=0.0,
                 rudder=0.0,
                 throttle_0=0.0,
                 throttle_1=0.0,
                 throttle_2=0.0,
                 throttle_3=0.0,
                 throttle_4=0.0):
        self.elevator = elevator  # elevator command
        self.aileron = aileron  # aileron command
        self.rudder = rudder  # rudder command
        self.throttle_0 = throttle_0  # throttle command
        self.throttle_1 = throttle_1
        self.throttle_2 = throttle_2
        self.throttle_3 = throttle_3
        self.throttle_4 = throttle_4

    def to_array(self):
        return np.array([[self.elevator],
                         [self.aileron],
                         [self.rudder],
                         [self.r4]])

    def from_array(self, u):
        self.elevator = u.item(0)
        self.aileron = u.item(1)
        self.rudder = u.item(2)
        self.throttle_4 = u.item(3)

    @property
    def throttles(self):
        return np.array([self.throttle_0, self.throttle_1, self.throttle_2, self.throttle_3, self.throttle_4])

    def print(self):
        print('elevator=', self.elevator,
              'aileron=', self.aileron,
              'rudder=', self.rudder,
              'throttle=', self.throttle_4)

