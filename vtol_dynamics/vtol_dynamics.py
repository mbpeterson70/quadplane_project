"""
vtolDynamics 
    - this file implements the dynamic equations of motion for VTOL
    - use unit quaternion for the attitude state
    
vtolsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/24/2020 - RWB
"""
import sys
sys.path.append('..')
import numpy as np

# load message types
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors
from message_types.msg_delta import MsgDelta

import parameters.quadplane_parameters as VTOL
import parameters.sensor_parameters as SENSOR
from tools.rotations import Quaternion2Rotation, Quaternion2Euler, Euler2Rotation

STATE_P_NORTH = 0
STATE_P_EAST = 1
STATE_P_DOWN = 2
STATE_V_U = 3
STATE_V_V = 4
STATE_V_W = 5
STATE_Q_E0 = 6
STATE_Q_E1 = 7
STATE_Q_E2 = 8
STATE_Q_E3 = 9
STATE_ANG_P = 10
STATE_ANG_Q = 11
STATE_ANG_R = 12

class VTOLDynamics:
    def __init__(self, Ts):
        self._ts_simulation = Ts
        # set initial states based on parameter file
        # _state is the 13x1 internal state of the aircraft that is being propagated:
        # _state = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r]
        # We will also need a variety of other elements that are functions of the _state and the wind.
        # self.true_state is a 19x1 vector that is estimated and used by the autopilot to control the aircraft:
        # true_state = [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        self._state = np.array([[VTOL.north0],  # (0)
                               [VTOL.east0],   # (1)
                               [VTOL.down0],   # (2)
                               [VTOL.u0],    # (3)
                               [VTOL.v0],    # (4)
                               [VTOL.w0],    # (5)
                               [VTOL.e0],    # (6)
                               [VTOL.e1],    # (7)
                               [VTOL.e2],    # (8)
                               [VTOL.e3],    # (9)
                               [VTOL.p0],    # (10)
                               [VTOL.q0],    # (11)
                               [VTOL.r0]])   # (12)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = VTOL.u0
        self._alpha = 0
        self._beta = 0
        # initialize true_state message
        self.true_state = MsgState()
        # initialize the sensors message
        self._sensors = MsgSensors()
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.
        # update velocity data and forces and moments
        self._update_velocity_data()
        self._forces_moments(delta=MsgDelta())


    ###################################
    # public functions
    def update(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        self._wind = wind
        # get forces and moments acting on rigid body
        forces_moments = self._forces_moments(delta)

        # print('dynamics: forces/moments')
        # print(forces_moments)

        # Integrate ODE using Runge-Kutta RK4 algorithm
        time_step = self._ts_simulation
        k1 = self._derivatives(self._state, forces_moments)
        k2 = self._derivatives(self._state + time_step/2.*k1, forces_moments)
        k3 = self._derivatives(self._state + time_step/2.*k2, forces_moments)
        k4 = self._derivatives(self._state + time_step*k3, forces_moments)
        self._state += time_step/6 * (k1 + 2*k2 + 2*k3 + k4)

        # normalize the quaternion
        e0 = self._state.item(6)
        e1 = self._state.item(7)
        e2 = self._state.item(8)
        e3 = self._state.item(9)
        normE = np.sqrt(e0**2+e1**2+e2**2+e3**2)
        self._state[6][0] = self._state.item(6)/normE
        self._state[7][0] = self._state.item(7)/normE
        self._state[8][0] = self._state.item(8)/normE
        self._state[9][0] = self._state.item(9)/normE

        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    def sensors(self):
        "Return value of sensors on VTOL: gyros, accels, absolute_pressure, dynamic_pressure, GPS"
        phi, theta, psi = Quaternion2Euler(self._state[STATE_Q_E0:STATE_Q_E3+1,:])
        pdot = Quaternion2Rotation(self._state[STATE_Q_E0:STATE_Q_E3+1,:]) @ \
            self._state[STATE_V_U:STATE_V_W+1,:]
        # simulate rate gyros(units are rad / sec)
        self._sensors.gyro_x = self._state.item(STATE_ANG_P) + SENSOR.gyro_x_bias \
            + np.random.normal(0, SENSOR.gyro_sigma)
        self._sensors.gyro_y = self._state.item(STATE_ANG_Q) + SENSOR.gyro_y_bias \
            + np.random.normal(0, SENSOR.gyro_sigma)
        self._sensors.gyro_z = self._state.item(STATE_ANG_R) + SENSOR.gyro_z_bias \
            + np.random.normal(0, SENSOR.gyro_sigma)
        # simulate accelerometers(units of g)
        self._sensors.accel_x = self._forces.item(0)/VTOL.mass + VTOL.gravity*np.sin(theta) \
            + np.random.normal(0, SENSOR.accel_sigma)
        self._sensors.accel_y = self._forces.item(1)/VTOL.mass \
            + VTOL.gravity*np.cos(theta)*np.sin(phi) + np.random.normal(0, SENSOR.accel_sigma)
        self._sensors.accel_z = self._forces.item(2)/VTOL.mass \
            + VTOL.gravity*np.cos(theta)*np.cos(phi) + np.random.normal(0, SENSOR.accel_sigma)
        # simulate magnetometers
        # magnetic field in provo has magnetic declination of 12.5 degrees
        # and magnetic inclination of 66 degrees
        R_mag = Euler2Rotation(0, -SENSOR.mag_inclination, SENSOR.mag_declination)
        # magnetic field in inertial frame: unit vector
        mag_inertial = R_mag.T @ np.array([[1, 0, 0]]).T
        R = Euler2Rotation(phi, theta, psi) # body to inertial
        # magnetic field in body frame: unit vector
        mag_body = R @ mag_inertial
        self._sensors.mag_x = mag_body.item(0) + np.random.normal(SENSOR.mag_beta, SENSOR.mag_sigma)
        self._sensors.mag_y = mag_body.item(1) + np.random.normal(SENSOR.mag_beta, SENSOR.mag_sigma)
        self._sensors.mag_z = mag_body.item(2) + np.random.normal(SENSOR.mag_beta, SENSOR.mag_sigma)
        # simulate pressure sensors
        self._sensors.abs_pressure = -VTOL.rho*VTOL.gravity*self._state.item(STATE_P_DOWN) \
            + np.random.normal(0, SENSOR.abs_pres_sigma)
        self._sensors.diff_pressure = (VTOL.rho*self._Va**2)/2 \
            + np.random.normal(0, SENSOR.diff_pres_sigma)
        # simulate GPS sensor
        if self._t_gps >= SENSOR.ts_gps:
            self._gps_eta_n = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_n \
                + SENSOR.ts_gps*np.random.normal(0, SENSOR.gps_n_sigma)
            self._gps_eta_e = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_e \
                + SENSOR.ts_gps*np.random.normal(0, SENSOR.gps_e_sigma)
            self._gps_eta_h = np.exp(-SENSOR.gps_k*SENSOR.ts_gps)*self._gps_eta_h \
                + SENSOR.ts_gps*np.random.normal(0, SENSOR.gps_h_sigma)
            self._sensors.gps_n = self._state.item(STATE_P_NORTH) + self._gps_eta_n
            self._sensors.gps_e = self._state.item(STATE_P_EAST) + self._gps_eta_e
            self._sensors.gps_h = -self._state.item(STATE_P_DOWN) + self._gps_eta_h
            self._sensors.gps_Vg = \
                np.sqrt((self._Va*np.cos(psi)+self._wind.item(0))**2 + \
                (self._Va*np.sin(psi)+self._wind.item(1))**2) \
                + np.random.normal(0, SENSOR.gps_Vg_sigma)
            self._sensors.gps_course = \
                np.arctan2(self._Va*np.sin(psi) + self._wind.item(1),
                (self._Va*np.cos(psi)+self._wind.item(0))) \
                + np.random.normal(0, SENSOR.gps_course_sigma)
            self._t_gps = 0.
        else:
            self._t_gps += self._ts_simulation
        return self._sensors

    def external_set_state(self, new_state):
        self._state = new_state

    ###################################
    # private functions
    def _derivatives(self, state, forces_moments):
        """
        for the dynamics xdot = f(x, u), returns f(x, u)
        """
        # extract the states
        # north = state.item(0)
        # east = state.item(1)
        # down = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments
        fx = forces_moments.item(0)
        fy = forces_moments.item(1)
        fz = forces_moments.item(2)
        l = forces_moments.item(3)
        m = forces_moments.item(4)
        n = forces_moments.item(5)

        # position kinematics
        pos_dot = Quaternion2Rotation(state[6:10,:]) @ state[3:6,:]
        north_dot = pos_dot.item(0)
        east_dot = pos_dot.item(1)
        down_dot = pos_dot.item(2)

        # position dynamics
        u_dot = r*v-q*w + (1/VTOL.mass) * fx
        v_dot = p*w-r*u + (1/VTOL.mass) * fy
        w_dot = q*u-p*v + (1/VTOL.mass) * fz

        # rotational kinematics
        e_dot_matrix = np.array([[0, -p, -q, -r],
                                 [p, 0, r, -q],
                                 [q, -r, 0, p],
                                 [r, q, -p, 0]])
        e_dot = 1/2 * e_dot_matrix @ state[6:10,:]
        e0_dot = e_dot.item(0)
        e1_dot = e_dot.item(1)
        e2_dot = e_dot.item(2)
        e3_dot = e_dot.item(3)

        # rotatonal dynamics
        p_dot = VTOL.gamma1*p*q - VTOL.gamma2*q*r + VTOL.gamma3*l + VTOL.gamma4*n
        q_dot = VTOL.gamma5*p*r - VTOL.gamma6*(p**2-r**2) + 1/VTOL.Jy*m
        r_dot = VTOL.gamma7*p*q - VTOL.gamma1*q*r + VTOL.gamma4*l + VTOL.gamma8*n

        # collect the derivative of the states
        x_dot = np.array([[north_dot, east_dot, down_dot, u_dot, v_dot, w_dot,
                           e0_dot, e1_dot, e2_dot, e3_dot, p_dot, q_dot, r_dot]]).T
        return x_dot

    def _update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]
        # convert wind vector from world to body frame
        wind_body_frame = Quaternion2Rotation(self._state[6:10,:]) @ steady_state + gust
        # velocity vector relative to the airmass
        v_g = self._state[3:6,:]
        self.v_air = v_g - wind_body_frame
        ur = self.v_air.item(0)
        vr = self.v_air.item(1)
        wr = self.v_air.item(2)
        # compute airspeed
        self._Va = np.sqrt(ur**2 + vr**2 + wr**2)
        # compute angle of attack
        if ur == 0:
            self._alpha = 0
        else:
            self._alpha = np.arctan(wr/ur)
        # compute sideslip angle
        if self._Va == 0:
            self._beta = np.inf
        else:
            self._beta = np.arcsin(vr / np.sqrt(ur**2+vr**2+wr**2))

    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)

        # compute gravitaional forces
        e0 = self._state.item(6)
        ex = self._state.item(7)
        ey = self._state.item(8)
        ez = self._state.item(9)
        f_g = VTOL.mass*VTOL.gravity * np.array([[2*(ex*ez-ey*e0)],
                                               [2*(ey*ez+ex*e0)],
                                               [ez**2+e0**2-ex**2-ey**2]])     

        # compute Lift and Drag coefficients
        sigma = (1 + np.exp(-VTOL.M*(self._alpha-VTOL.alpha0)) + np.exp(VTOL.M*(self._alpha+VTOL.alpha0))) / \
            ((1 + np.exp(-VTOL.M*(self._alpha-VTOL.alpha0))) * (1 + np.exp(VTOL.M*(self._alpha+VTOL.alpha0)))) 
        CL = (1-sigma)*(VTOL.C_L_0+VTOL.C_L_alpha*self._alpha) + \
            sigma*2*np.sign(self._alpha)*(np.sin(self._alpha)**2)*np.cos(self._alpha)
        CD = VTOL.C_D_p + ((VTOL.C_L_0 + VTOL.C_L_alpha*self._alpha)**2) / (np.pi*VTOL.e*VTOL.AR)
        # compute Lift and Drag Forces
        Gamma_Va = (1/2)*VTOL.rho*(self._Va**2)*VTOL.S_wing
        F_lift = (1/2)*VTOL.rho*(self._Va**2)*VTOL.S_wing*(CL + VTOL.C_L_q*VTOL.c/(2*self._Va)*q \
            + VTOL.C_L_delta_e*delta.elevator)
        F_drag = (1/2)*VTOL.rho*(self._Va**2)*VTOL.S_wing*\
            (CD + VTOL.C_D_q*VTOL.c/(2*self._Va)*q + VTOL.C_D_delta_e*delta.elevator)

        #compute propeller thrust and torque
        prop_airspeeds = np.array([-self.v_air.item(2), -self.v_air.item(2), 
            -self.v_air.item(2), -self.v_air.item(2), self.v_air.item(0)])
        throttles = delta.throttles
        thrust_prop, torque_prop = self._motor_thrust_torque(prop_airspeeds, throttles, VTOL.prop_dirs)


        # compute longitudinal forces in body frame
        f_body = np.array([[np.cos(self._alpha), -np.sin(self._alpha)],
                           [np.sin(self._alpha), np.cos(self._alpha)]]) @ \
                               np.array([[-F_drag], [-F_lift]])
        fx = f_body.item(0) + f_g.item(0) + thrust_prop[4]
        fz = f_body.item(1) + f_g.item(2) - \
            (thrust_prop[0] + thrust_prop[1] + thrust_prop[2] + thrust_prop[3])

        # compute lateral forces in body frame
        fy = (1/2)*VTOL.rho*(self._Va**2)*VTOL.S_wing * \
            (VTOL.C_Y_0 + VTOL.C_Y_beta*self._beta \
            + VTOL.C_Y_p*VTOL.b/(2*self._Va)*p + VTOL.C_Y_r*VTOL.b/(2*self._Va)*r \
            + VTOL.C_Y_delta_a*delta.aileron + VTOL.C_Y_delta_r*delta.rudder) \
            + f_g.item(1)

        My_rotors = (thrust_prop[0]*VTOL.rotor_q0.item(0) + thrust_prop[1]*VTOL.rotor_q1.item(0)
            + thrust_prop[2]*VTOL.rotor_q2.item(0) + thrust_prop[3]*VTOL.rotor_q3.item(0))
        Mx_rotors = -(thrust_prop[0]*VTOL.rotor_q0.item(1) + thrust_prop[1]*VTOL.rotor_q1.item(1)
            + thrust_prop[2]*VTOL.rotor_q2.item(1) + thrust_prop[3]*VTOL.rotor_q3.item(1)
            + torque_prop[4])
        Mz_rotors = torque_prop[0] + torque_prop[1] + torque_prop[2] + torque_prop[3]

        # compute logitudinal torque in body frame
        My = (1/2)*VTOL.rho*(self._Va**2)*VTOL.S_wing*VTOL.c * \
            (VTOL.C_m_0 + VTOL.C_m_alpha*self._alpha + VTOL.C_m_q*VTOL.c/(2*self._Va)*q \
            + VTOL.C_m_delta_e*delta.elevator)

        # compute lateral torques in body frame
        Mx = (1/2)*VTOL.rho*(self._Va**2)*VTOL.S_wing*VTOL.b * \
            (VTOL.C_ell_0 + VTOL.C_ell_beta*self._beta \
            + VTOL.C_ell_p*VTOL.b/(2*self._Va)*p + VTOL.C_ell_r*VTOL.b/(2*self._Va)*r \
            + VTOL.C_ell_delta_a*delta.aileron + VTOL.C_ell_delta_r*delta.rudder)
        Mz = (.5)*VTOL.rho*(self._Va**2)*VTOL.S_wing*VTOL.b * \
            (VTOL.C_n_0 + VTOL.C_n_beta*self._beta \
            + VTOL.C_n_p*VTOL.b/(2*self._Va)*p + VTOL.C_n_r*VTOL.b/(2*self._Va)*r \
            + VTOL.C_n_delta_a*delta.aileron + VTOL.C_n_delta_r*delta.rudder)
        
        Mx += Mx_rotors
        My += My_rotors
        Mz += Mz_rotors

        self._forces[0] = fx
        self._forces[1] = fy
        self._forces[2] = fz
        return np.array([[fx, fy, fz, Mx, My, Mz]]).T

    def _motor_thrust_torque(self, Va, delta_t, direction):
        # compute thrust and torque due to propeller
        # map delta_t throttle command(0 to 1) into motor input voltage
        V_in = VTOL.V_max*delta_t

        # Angular speed of propeller
        a = VTOL.rho*(VTOL.D_prop**5) / ((2*np.pi)**2) * VTOL.C_Q0
        b = (VTOL.rho*(VTOL.D_prop**4) / (2*np.pi) * VTOL.C_Q1*Va) + (VTOL.KQ**2/VTOL.R_motor)
        c = VTOL.rho*(VTOL.D_prop**3)*VTOL.C_Q2*(Va**2) - (VTOL.KQ/VTOL.R_motor)*V_in + VTOL.KQ*VTOL.i0
        Omega_p = (-b + np.sqrt(b**2 - 4*a*c)) / (2*a)

        # thrust and torque due to propeller
        thrust_prop = VTOL.rho*(VTOL.D_prop**4)*VTOL.C_T0 / (4*(np.pi**2)) * Omega_p**2 \
            + VTOL.rho*(VTOL.D_prop**3)*VTOL.C_T1*Va / (2*np.pi) * Omega_p \
            + VTOL.rho*(VTOL.D_prop**2)*VTOL.C_T2*(Va**2)
        torque_prop = -direction * (VTOL.rho*(VTOL.D_prop**5)*VTOL.C_Q0 / (4*(np.pi**2)) * Omega_p**2 \
            + VTOL.rho*(VTOL.D_prop**4)*VTOL.C_Q1*Va / (2*np.pi) * Omega_p \
            + VTOL.rho*(VTOL.D_prop**3)*VTOL.C_Q2*(Va**2))
        return thrust_prop, torque_prop

    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
        phi, theta, psi = Quaternion2Euler(self._state[6:10])
        pdot = Quaternion2Rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
        self.true_state.bx = SENSOR.gyro_x_bias
        self.true_state.by = SENSOR.gyro_y_bias
        self.true_state.bz = SENSOR.gyro_z_bias
