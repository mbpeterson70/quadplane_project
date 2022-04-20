from curses.ascii import ctrl
import re
from socket import AI_ALL
import sys
sys.path.append('..')
import numpy as np
from scipy.optimize import linprog, minimize, NonlinearConstraint

import parameters.quadplane_parameters as VTOL
import parameters.control_allocation_parameters as CA
from message_types.msg_delta import MsgDelta
from tools.rotations import Quaternion2Euler

CA_ROTOR_FRONT_RIGHT = 0
CA_ROTOR_FRONT_LEFT = 1
CA_ROTOR_BACK_RIGHT = 2
CA_ROTOR_BACK_LEFT = 3
CA_ROTOR_PULLER = 4
CA_ELEVATOR = 5
CA_AILERON = 6
CA_RUDDER = 7

class NonlinearControlAllocation():
    def __init__(self):
        self.previous_solution = CA.init_actuators
        self.max_iter = CA.max_iter
        self.actuator_bounds = [(0.0,  1.0),
                                (0.0,  1.0),
                                (0.0,  1.0),
                                (0.0,  1.0),
                                (0.0,  1.0),
                                (-1.0, 1.0),
                                (-1.0, 1.0),
                                (-1.0, 1.0)]

    def update(self, thrust, torques, state, airspeed):

        thrust_torque_desired = np.concatenate([thrust, torques], axis=0).reshape(-1)
        v_body = state[3:6]

        actuator_commands = self._compute_nonlinear_optimization(thrust_torque_desired, v_body, airspeed)

        return self._formulate_ctrl_msg(actuator_commands)

    def _compute_nonlinear_optimization(self, thrust_torque_desired, v_body, airspeed):

        x0 = self.previous_solution
        
        # Non linear optimizer gets optimization output and gradient from nonlinear_ctrl_optimization output
        res = minimize(
            nonlinear_ctrl_optimization, 
            x0,
            args=(thrust_torque_desired, v_body, airspeed, x0),
            bounds=self.actuator_bounds,
            jac=True,
            options={'maxiter': self.max_iter})
        self.previous_solution = res.x
        # print(res.x)
        # print('ca:')
        # print(thrust_torque_desired)
        # print(calc_thrust_torque_achieved(res.x, v_body, airspeed))
        return res.x

    def _formulate_ctrl_msg(self, actuator_commands):
        ctrl_msg = MsgDelta()

        ctrl_msg.throttle_0 = actuator_commands[CA_ROTOR_FRONT_RIGHT]
        ctrl_msg.throttle_1 = actuator_commands[CA_ROTOR_FRONT_LEFT]
        ctrl_msg.throttle_2 = actuator_commands[CA_ROTOR_BACK_RIGHT]
        ctrl_msg.throttle_3 = actuator_commands[CA_ROTOR_BACK_LEFT]
        ctrl_msg.throttle_4 = actuator_commands[CA_ROTOR_PULLER]

        ctrl_msg.elevator = actuator_commands[CA_ELEVATOR]
        ctrl_msg.aileron = actuator_commands[CA_AILERON]
        ctrl_msg.rudder = actuator_commands[CA_RUDDER]
        return ctrl_msg

# Possible deltas are fed into this function as x and are optimized to provide smallest 
# difference between commanded and achieved thrusts and torques
def nonlinear_ctrl_optimization(x, thrust_torque_desired, v_body, airspeed, prev_solution):
    K_Tau = CA.K_Tau
    K_delta = CA.K_delta(airspeed)
    x_des = CA.actuators_desired

    # compute the thrust/torque and its derivative with respect to the change of throttle
    Va_lift = v_body.item(2)
    Va_pull = v_body.item(0)

    thrust, torque, thrust_der, torque_der = \
        rotor_thrust_torque_der(x[CA_ROTOR_FRONT_RIGHT:CA_ROTOR_PULLER + 1], 
        [Va_lift, Va_lift, Va_lift, Va_lift, Va_pull],
        VTOL.prop_dirs.tolist())

    # Compute elevon forces
    elevator_force_coefs = calc_elevator_force(v_body, airspeed)

    thrust_torque_achieved = _calc_thrust_torque_achieved(
        x, thrust, torque, elevator_force_coefs, airspeed, v_body)
    thrust_torque_diff = thrust_torque_desired - thrust_torque_achieved
    diff_norm = 0.5 * thrust_torque_diff.T @ K_Tau @ thrust_torque_diff \
        + .5 * (x - x_des).T @ K_delta @ (x - x_des)

    thrust_torque_der = calc_thrust_torque_achieved_der(
        x, thrust, torque, thrust_der, torque_der, elevator_force_coefs, airspeed)
    diff_norm_der = -thrust_torque_der @ K_Tau @ thrust_torque_diff \
        + K_delta @ (x - x_des)
    
    return diff_norm, diff_norm_der

# Returns the thrust and torque achieved by certain deltas
# x is the proposed setpoint for each of the actuators
# thrust is a three-dimensional vector of thrust achieved by each of the three rotors
# torque is a three-dimensional vector representing the torque caused by each of the 
# rotors
def _calc_thrust_torque_achieved(x, thrust, torque, elevator_force_coefs, airspeed, v_body):

    Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing

    T_x = thrust[4] + elevator_force_coefs[0] * (x[CA_ELEVATOR])
    T_z = -(thrust[0] + thrust[1] + thrust[2] + thrust[3]) \
        + elevator_force_coefs[0] * (x[CA_ELEVATOR])
    Tau_x = torque[4] + Gamma * VTOL.b * VTOL.C_ell_delta_a * x[CA_AILERON] \
        + Gamma * VTOL.b * VTOL.C_ell_delta_r * x[CA_RUDDER]
    for i in range(4):
        Tau_x -= VTOL.rotor_qs[i].item(1) * thrust[i]
    Tau_y = Gamma * VTOL.c * VTOL.C_m_delta_e * x[CA_ELEVATOR]
    for i in range(4):
        Tau_y += VTOL.rotor_qs[i].item(0) * thrust[i]
    Tau_z = torque[0] + torque[1] + torque[2] + torque[3] \
        + Gamma * VTOL.b * \
        (VTOL.C_n_delta_a*x[CA_AILERON] + VTOL.C_n_delta_r*x[CA_RUDDER])

    return np.array([T_x, T_z, Tau_x, Tau_y, Tau_z]).T

# Calculates the gradient of the thrust and torque achieved
def calc_thrust_torque_achieved_der(
    x, thrust, torque, thrust_der, torque_der, elevator_force_coef, airspeed):

    Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing


    T_x_der =  [0.,
                0.,
                0.,
                0.,
                thrust_der[4],
                elevator_force_coef[0],
                0.,
                0.]
    T_z_der = [-thrust_der[0],
                -thrust_der[1],
                -thrust_der[2],
                -thrust_der[3],
                0.,
                elevator_force_coef[1],
                0.,
                0.]
    Tau_x_der = [-VTOL.rotor_q0.item(1) * thrust_der[0],
                -VTOL.rotor_q1.item(1) * thrust_der[1],
                -VTOL.rotor_q2.item(1) * thrust_der[2],
                -VTOL.rotor_q3.item(1) * thrust_der[3],
                torque_der[4],
                0.0,
                Gamma * VTOL.b * VTOL.C_ell_delta_a,
                Gamma * VTOL.b * VTOL.C_ell_delta_r]
    Tau_y_der = [VTOL.rotor_q0.item(0) * thrust_der[0],
                VTOL.rotor_q1.item(0) * thrust_der[1],
                VTOL.rotor_q2.item(0) * thrust_der[2],
                VTOL.rotor_q3.item(0) * thrust_der[3],
                0.0,
                Gamma * VTOL.c * VTOL.C_m_delta_e,
                0.0,
                0.0]
    Tau_z_der = [torque_der[0],
                torque_der[1],
                torque_der[2],
                torque_der[3],
                0.0,
                0.0,
                Gamma * VTOL.b * VTOL.C_n_delta_a,
                Gamma * VTOL.b * VTOL.C_n_delta_r]
    return np.array([T_x_der, T_z_der, Tau_x_der, Tau_y_der, Tau_z_der]).T

def rotor_thrust_torque_der(delta, Va, dir):
    thrust = list()
    torque = list()
    thrust_der = list()
    torque_der = list()
    for i in range(5):
        # compute thrust and torque due to propeller  (See addendum by McLain)
        # grab motor/prop params
        C_Q0 = VTOL.C_Q0
        C_Q1 = VTOL.C_Q1
        C_T0 = VTOL.C_T0
        C_Q2 = VTOL.C_Q2
        C_T1 = VTOL.C_T1
        C_T2 = VTOL.C_T2
        D_prop = VTOL.D_prop
        KQ = VTOL.KQ
        R_motor = VTOL.R_motor
        i0 = VTOL.i0
        # map delta_t throttle command(0 to 1) into motor input voltage
        V_in = VTOL.V_max * delta[i]
        V_in_der = VTOL.V_max
        # Quadratic formula to solve for motor speed
        a = C_Q0 * VTOL.rho * np.power(D_prop, 5) \
            / ((2.*np.pi)**2)
        b = (C_Q1 * VTOL.rho * np.power(D_prop, 4)
            / (2.*np.pi)) * Va[i] + KQ**2/R_motor
        c = C_Q2 * VTOL.rho * np.power(D_prop, 3) \
            * (Va[i])**2 - (KQ / R_motor) * V_in + KQ * i0
        c_der = (KQ / R_motor) * V_in_der
        # Consider only positive root
        Omega_op = (-b + np.sqrt(b**2 - 4*a*c)) / (2.*a)
        Omega_op_der = c_der / np.sqrt(b**2 - 4*a*c)
        # compute advance ratio
        J_op = 2 * np.pi * Va[i] / (Omega_op * D_prop)
        J_op_der = -2 * np.pi * Va[i] * Omega_op_der / (Omega_op**2 * D_prop)
        # compute non-dimensionalized coefficients of thrust and torque
        C_T = C_T2 * J_op**2 + C_T1 * J_op + C_T0
        C_Q = C_Q2 * J_op**2 + C_Q1 * J_op + C_Q0
        C_T_der = 2 * C_T2 * J_op * J_op_der + C_T1 * J_op_der
        C_Q_der = 2 * C_Q2 * J_op * J_op_der + C_Q1 * J_op_der
        # add thrust and torque due to propeller
        n = Omega_op / (2 * np.pi)
        T_p = VTOL.rho * n**2 * np.power(D_prop, 4) * C_T
        Q_p = VTOL.rho * n**2 * np.power(D_prop, 5) * C_Q
        T_p_der = VTOL.rho * Omega_op * Omega_op_der * np.power(D_prop, 4) * C_T / (2 * np.pi**2) + \
            VTOL.rho * Omega_op**2 * np.power(D_prop, 4) * C_T_der / (2 * np.pi)**2
        Q_p_der = VTOL.rho * Omega_op * Omega_op_der * np.power(D_prop, 5) * C_Q / (2 * np.pi**2) + \
            VTOL.rho * Omega_op**2 * np.power(D_prop, 5) * C_Q_der / (2 * np.pi)**2
        # Flip moment sign for certain rotors
        Q_p *= -dir[i]
        Q_p_der *= -dir[i]

        thrust.append(T_p)
        torque.append(Q_p)
        thrust_der.append(T_p_der)
        torque_der.append(Q_p_der)

    return thrust, torque, thrust_der, torque_der

def calc_elevator_force(v_body, airspeed):
    Gamma = .5 * VTOL.rho * airspeed**2 * VTOL.S_wing
    elevator_lift_coef = Gamma * VTOL.C_L_delta_e
    elevator_drag_coef = Gamma * VTOL.C_D_delta_e
    if v_body[0] != 0:
        alpha = np.arctan2(v_body[2], v_body[0]).item(0)
    else:
        alpha = 0
    elevator_force_coefs = [-np.cos(alpha) * elevator_drag_coef + np.sin(alpha) * elevator_lift_coef,
                          -np.sin(alpha) * elevator_drag_coef - np.cos(alpha) * elevator_lift_coef]
    return elevator_force_coefs

def calc_thrust_torque_achieved(x, v_body, airspeed):

    Va_lift = v_body.item(2)
    Va_pull = v_body.item(0)

    thrust, torque, thrust_der, torque_der = \
        rotor_thrust_torque_der(x[CA_ROTOR_FRONT_RIGHT:CA_ROTOR_PULLER + 1], 
        [Va_lift, Va_lift, Va_lift, Va_lift, Va_pull],
        VTOL.prop_dirs.tolist())

    elevator_force_coefs = calc_elevator_force(v_body, airspeed)

    return _calc_thrust_torque_achieved(x, thrust, torque, elevator_force_coefs, airspeed, v_body)
