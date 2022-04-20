#/usr/bin/python3

import sys

sys.path.append('..')
sys.path.append('../viewers')
sys.path.append('../trajectorygenerator/scripts')
import numpy as np
import time
import parameters.simulation_parameters as SIM

from vtol_viewer.vtol_viewer import VTOLViewer
from chap3.data_viewer import DataViewer
from vtol_dynamics.vtol_dynamics import VTOLDynamics
from chap4.wind_simulation import WindSimulation
from message_types.msg_delta import MsgDelta
from message_types.msg_state import MsgState
from vtol_control_allocation.nonlinear_control_allocation import NonlinearControlAllocation
from low_level_controller.rate_control import RateControl
from pitch_free_trajectory_tracker import PitchFreeTrajectoryTracker
from pitch_control import PitchControl
from attitude_control import AttitudeControl
from tools.msg_convert import *
from tools.rotations import Quaternion2Euler, Rotation2Quaternion, Rotation2Euler, Quaternion2Rotation

# trajectories
import vtol_trajectory_generator.trajectories as TRAJ


def main():
    np.set_printoptions(precision=4, linewidth=200, suppress=True)
    # initialize viewers
    state_view = DataViewer()
    vtol_view = VTOLViewer()

    # initialize elements of the architecture
    wind = WindSimulation(SIM.ts_simulation)
    vtol = VTOLDynamics(SIM.ts_simulation)

    # INITIALIZE TRAJECTORIES
    traj = TRAJ.tcl
    
    ## ---------------------------------

    # draw the trajectory
    SIM.end_time = traj.end_time
    trajectory_position_points = traj.get_position_pts(.01)

    vtol_view.addTrajectory(trajectory_position_points[:3,:])

    # initialize geometric controller
    traj_tracker = PitchFreeTrajectoryTracker()
    att_ctrl = AttitudeControl()
    pitch_ctrl = PitchControl()

    #initialize low level control
    rate_control = RateControl(ts_control=SIM.ts_control)
    control_alloc = NonlinearControlAllocation()

    # initialize command message
    delta = MsgDelta()

    #calculate_trim
    vtol._update_true_state()

    # initialize the simulation time
    sim_time = SIM.start_time
    Ts = SIM.ts_simulation

    time_hist = []
    comp_time_hist = []


    # main simulation loop
    while sim_time < SIM.end_time:
        #-------observer-------------
        measurements = vtol.sensors()  # get sensor measurements
        estimated_state = vtol._state  # estimated state is current state

        ctrl_start_time = time.time()
        # ------ Trajectory follower
        traj_derivatives_at_t = traj.traj_msg(sim_time)

        #------- High Level controller-------------
        T, R_d = traj_tracker.update(estimated_state, traj_derivatives_at_t)
        T, R_d = pitch_ctrl.update(T, R_d, estimated_state[3:6])
        T = T.reshape(-1)
        omega_c = att_ctrl.update(Quaternion2Rotation(estimated_state[6:10]), R_d)
        omega_c = omega_c.reshape(-1)

        #------- Low Level Controller -------------
        omega = estimated_state[10:13,0]
        tau_c = rate_control.update(omega_c, omega)
        delta = control_alloc.update(T, tau_c, estimated_state, vtol._Va)
        ctrl_end_time = time.time()

        #-------update physical system-------------
        vtol.update(delta, np.array([[0.0,0.0,0.0,0.0,0.0,0.0]]).T)  # propagate the MAV dynamics

        #-------update viewers-------------
        pd_i = traj_derivatives_at_t[0:3,0]
        va_d = np.linalg.norm(traj_derivatives_at_t[0:3,1])
        desired_state = MsgState()
        desired_state.north = pd_i.item(0)
        desired_state.east = pd_i.item(1)
        desired_state.altitude = -pd_i.item(2)
        desired_state.Va = va_d
        desired_state.phi, desired_state.theta, desired_state.chi = Rotation2Euler(R_d)

        vtol_view.update(vtol.true_state)
        state_view.update(vtol.true_state, vtol.true_state, desired_state, delta, Ts)
        time_hist.append(sim_time)
        comp_time_hist.append(ctrl_end_time - ctrl_start_time)

        #-------increment time-------------
        sim_time += Ts

    print("Done with simulation")
    while (True):
        vtol_view.app.processEvents()
        state_view.plotter.app.processEvents()
    return

main()
