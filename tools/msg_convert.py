"""
    convert messages to/from numpy arrays
"""
import sys
sys.path.append('..')
import numpy as np
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState

def np2msg_controls(delta):
        msg = MsgDelta()
        msg.throttle_0 = delta.item(0) 
        msg.throttle_1 = delta.item(1)
        msg.throttle_2 = delta.item(2)
        msg.throttle_3 = delta.item(3)
        msg.throttle_4 = delta.item(4)
        msg.elevator = delta.item(5)
        msg.aileron = delta.item(6)
        msg.rudder = delta.item(7)
        return msg

def msg_controls2np(msg):
    return np.array([[ msg.throttle_0, msg.throttle_1, \
        msg.throttle_2, msg.throttle_3, msg.throttle_4, \
        msg.elevator, msg.aileron, msg.rudder ]]).T

def msg_state2np(msg):
        return np.array([[msg.pn],  # [0]  north position
                        [msg.pe],   # [1]  east position
                        [-msg.h],   # [2]  down position
                        [msg.u],    # [3]  velocity along body x-axis
                        [msg.v],    # [4]  velocity along body y-axis
                        [msg.w],    # [5]  velocity along body z-axis
                        [msg.phi],  # [6]  pitch angle
                        [msg.theta],# [7]  roll angle
                        [msg.psi],  # [8]  yaw angle
                        [msg.p],   # [9]  roll rate
                        [msg.q],   # [10]  pitch rate
                        [msg.r],   # [11]  yaw rate
                        ])


def np2msg_state(x):
    msg = MsgState()
    msg.north       = x.item(0) # [0]  north position
    msg.east        = x.item(1) # [1]  east position
    msg.altitude    = -x.item(2) # [2]  down position
    msg.u    = x.item(3) # [3]  velocity along body x-axis
    msg.v    = x.item(4) # [4]  velocity along body y-axis
    msg.w    = x.item(5) # [5]  velocity along body z-axis
    msg.phi  = x.item(6) # [6]  pitch angle
    msg.theta= x.item(7) # [7]  roll angle
    msg.psi  = x.item(8) # [8]  yaw angle
    msg.p    = x.item(9) # [9]  roll rate
    msg.q    = x.item(10) # [10]  pitch rate
    msg.r    = x.item(11) # [11]  yaw rate
    
    return msg



