import datetime
import math
import random
import time

from flightgear_python.fg_if import FDMConnection, CtrlsConnection
from numpy import ndarray
import numpy as np
from pid_control_hierarchy_fdm import AircraftController, PIDController
from skopt import gp_minimize
from skopt.space import Real

child_aileron_state = 0.0
child_elevator_state = 0.0
child_rudder_state = 0.0

n = 1


def ctrls_callback(ctrls_data, event_pipe):
    global child_aileron_state
    global child_rudder_state
    global child_elevator_state

    if event_pipe.child_poll():
        child_aileron_req, child_rudder_req, child_elevator_req = event_pipe.child_recv()  # Unpack tuple from parent

        child_aileron_state = child_aileron_req
        child_rudder_state = child_rudder_req
        child_elevator_state = child_elevator_req

    ctrls_data.aileron = child_aileron_state  # from -1..1
    ctrls_data.rudder = child_rudder_state
    ctrls_data.elevator = child_elevator_state
    return ctrls_data


def fdm_callback(fdm_data, event_pipe):
    if event_pipe.child_poll():
        child_start_new, = event_pipe.child_recv()
        if child_start_new:
            start_alitude_ft = random.randint(2000, 4000) * 0.3048
            fdm_data.alt_m = start_alitude_ft
            fdm_data.phi_rad = 0
            fdm_data.climb_rate_ft_per_s = 0
            fdm_data.theta_rad = 0
            fdm_data.vcas = 120
            child_start_new = False
            return fdm_data
    roll_deg = math.degrees(fdm_data.phi_rad)
    climb_rate_ft_per_s = fdm_data.climb_rate_ft_per_s
    alt_m = fdm_data.alt_m
    alpha_rad = math.degrees(fdm_data.alpha_rad)
    psi_rad = math.degrees(fdm_data.psi_rad)
    elevator = fdm_data.elevator
    event_pipe.child_send((roll_deg, climb_rate_ft_per_s, alt_m, alpha_rad, psi_rad, elevator))


def maneuvering(roc_kp, roc_ki, roc_kd, pitch_kp, pitch_ki, pitch_kd, elevator_kp, elevator_ki, elevator_kd) -> ndarray:
    global n

    start_new = True
    fdm_event_pipe.parent_send((start_new,))
    time.sleep(1)
    # Создаем экземпляры классов
    aircraft_controller = AircraftController(roc_kp, roc_ki, roc_kd, pitch_kp, pitch_ki, pitch_kd, elevator_kp, elevator_ki, elevator_kd)

    target_altitude = random.randint(2000, 4000)


    # DATA COLLECTION INIT
    altitude_error_list = []
    vertical_speed_dif_list = []
    altitude_list = []
    elevator_deflection_list = []
    roc_list = []
    target_roc_list = []

    # TIMER
    start_time = time.time()

    roll_Kp = 0.01
    head_Kp = 0.01
    roll_deg_setpoint = 0
    head_deg_setpoint = 180

    _, last_vertical_speed, start_altitude_m, _, _, _ = fdm_event_pipe.parent_recv()
    start_altitude_ft = start_altitude_m * 3.28084
    print(f"Start: {start_altitude_ft} - Target: {target_altitude}")
    print(f"roc_kp: {roc_kp} roc_ki: {roc_ki} roc_kd: {roc_kd}\n"
          f"pitch_kp: {pitch_kp} pitch_ki: {pitch_ki} pitch_kd: {pitch_kd}\n"
          f"elevator_kp: {elevator_kp} elevator_ki: {elevator_ki} elevator_kd: {elevator_kd}")

    while time.time() - start_time < 120:
        if fdm_event_pipe.parent_poll():
            # RECEIVING DATA FROM fdm_callback
            parent_roll_deg, parent_climb_rate_ft_per_s, parent_alt_m, parent_alpha_rad, parent_psi_rad, parent_elevator = fdm_event_pipe.parent_recv()
            parent_alt_ft = parent_alt_m * 3.28084

            # AILERON | RUDDER P-CONTROLLER
            roll_error = roll_deg_setpoint - parent_roll_deg
            head_error = head_deg_setpoint - parent_psi_rad
            parent_aileron_req = roll_error * roll_Kp
            parent_rudder_req = max(min(head_error * head_Kp, 0.3), -0.3)

            # AIRCRAFT CONTROLLER
            parent_elevator_req = aircraft_controller.update(target_altitude, parent_alt_ft, parent_climb_rate_ft_per_s, parent_elevator)

            # SENDING SIGNALS TO ctrls_callback
            ctrls_event_pipe.parent_send((parent_aileron_req, parent_rudder_req, parent_elevator_req))

            # DATA COLLECTION
            altitude_list.append(parent_alt_ft)
            roc_list.append(parent_climb_rate_ft_per_s)
            elevator_deflection_list.append(parent_elevator_req)
            target_roc_list.append(aircraft_controller.target_roc) #FIXME
            # METRICS COLLECTION
            altitude_error = parent_alt_m - target_altitude
            altitude_error_list.append(altitude_error)

            vertical_speed_dif = parent_climb_rate_ft_per_s - last_vertical_speed
            vertical_speed_dif_list.append(vertical_speed_dif)
            last_vertical_speed = parent_climb_rate_ft_per_s
        if parent_alt_ft < 150:
            break

    # METRICS CALCULATION
    coefficient = max(20, abs(start_altitude_ft - target_altitude))
    l1_altitude_error = np.sum(np.abs(np.array(altitude_error_list) / coefficient))
    l1_vertical_speed_dif = np.sum(np.abs(np.array(vertical_speed_dif_list) / np.sqrt(coefficient)))

    # SAVE LOGS FIXME REWORK LOGS
    filename = f"data/{n}-Start={start_altitude_ft}-Target={target_altitude}-L1_AE={round(l1_altitude_error, 3)}-L1_VSD={round(l1_vertical_speed_dif, 3)}.txt"
    data = [altitude_list, roc_list, target_roc_list, altitude_error_list,
            vertical_speed_dif_list, elevator_deflection_list]
    np.savetxt(filename, np.column_stack(data), fmt="%.8f", delimiter=",", header="Altitude, VerticalSpeed, TargetVerticalSpeed, AltitudeError, VS_difference, Elevator")

    # DEBUG INFO
    print(f"L1 | altitude_error: {l1_altitude_error}")
    print(f"L1 | vertical_speed_dif: {l1_vertical_speed_dif}\n")
    n += 1
    return l1_altitude_error + l1_vertical_speed_dif


def objective(params):
    roc_kp, roc_ki, roc_kd, pitch_kp, pitch_ki, pitch_kd, elevator_kp, elevator_ki, elevator_kd = params
    return maneuvering(roc_kp, roc_ki, roc_kd, pitch_kp, pitch_ki, pitch_kd, elevator_kp, elevator_ki, elevator_kd)


if __name__ == "__main__":
    ctrls_conn = CtrlsConnection(ctrls_version=27)
    ctrls_event_pipe = ctrls_conn.connect_rx('localhost', 5503, ctrls_callback)
    ctrls_conn.connect_tx('localhost', 5504)

    fdm_conn = FDMConnection(fdm_version=24)
    fdm_event_pipe = fdm_conn.connect_rx('localhost', 5501, fdm_callback)
    fdm_conn.connect_tx('localhost', 5502)

    ctrls_conn.start()
    fdm_conn.start()

    space = [
        Real(-5, 1, name='roc_kp'),
        Real(-5, -1, name='roc_ki'),
        Real(-5, -1, name='roc_kd'),
        Real(-5, -1, name='pitch_kp'),
        Real(-5, -1, name='pitch_ki'),
        Real(-5, -1, name='pitch_kd'),
        Real(-5, -1, name='elevator_kp'),
        Real(-5, -1, name='elevator_ki'),
        Real(-5, -1, name='elevator_kd'),
    ]

    #maneuvering(-0.9807275087798706, -1.4815004639651028, -2.601725489049133, -3.0118800777735473, -3.816516655665921, -3.2859154875561467)

    result = gp_minimize(objective, space, n_calls=150, n_random_starts=1)
    print(f'Best parameters: {result.x}')
    print(f'Best score: {result.fun}')
