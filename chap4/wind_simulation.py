"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
import sys
sys.path.append('..')
from tools.transfer_function import transferFunction
import numpy as np
import parameters.wind_parameters as WIND


class WindSimulation:
    def __init__(self, Ts):
        gust_flag = WIND.gust_flag
        wind_flag = WIND.wind_flag

        # steady state wind defined in the inertial frame
        if wind_flag:
            self._steady_state = np.array([[3., 3., 0.]]).T
        else:
            self._steady_state = np.array([[0., 0., 0.]]).T  

        #   Dryden gust model parameters (section 4.4 UAV book)
        Va = 25 # must set Va to a constant value
        Lu = WIND.Lu
        Lv = WIND.Lv
        Lw = WIND.Lw
        if gust_flag==True:
            sigma_u = WIND.sigma_u
            sigma_v = WIND.sigma_v
            sigma_w = WIND.sigma_w
        else:
            sigma_u = 0
            sigma_v = 0
            sigma_w = 0

        # Dryden transfer functions (section 4.4 UAV book)
        H_u_coef = sigma_u*np.sqrt(2*Va/(np.pi*Lu))
        H_v_coef = sigma_v*np.sqrt(3*Va/(np.pi*Lv))
        H_w_coef = sigma_w*np.sqrt(3*Va/(np.pi*Lw))
        self.H_u = transferFunction(
            num=np.array([[H_u_coef]]), 
            den=np.array([[1, Va/Lu]]),
            Ts=Ts
        )
        self.H_v = transferFunction(
            num=np.array([[H_v_coef, H_v_coef*Va/(np.sqrt(3)*Lv)]]), 
            den=np.array([[1, 2*Va/Lv, Va**2/Lv**2]]),
            Ts=Ts
        )
        self.H_w = transferFunction(
            num=np.array([[H_w_coef, H_w_coef*Va/(np.sqrt(3)*Lw)]]), 
            den=np.array([[1, 2*Va/Lw, Va**2/Lw**2]]),
            Ts=Ts
        )
        self._Ts = Ts

    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        gust = np.array([[self.H_u.update(np.random.randn())],
                         [self.H_v.update(np.random.randn())],
                         [self.H_w.update(np.random.randn())]])
        return np.concatenate(( self._steady_state, gust ))

