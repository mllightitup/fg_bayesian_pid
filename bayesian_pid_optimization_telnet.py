import datetime
import random
import time
import pandas as pd

from pid_control_hierarchy_telnet import AircraftController
from pid_logger import pid_logger
from flightgear_utils_telnet import FGUtils
from flightgear_python.fg_if import PropsConnection

from numpy import ndarray
import numpy as np

from skopt import gp_minimize
from skopt.space import Real

n = 1


def maneuvering(roc_kp, roc_ki, roc_kd, pitch_kp, pitch_ki, pitch_kd, elevator_kp, elevator_ki, elevator_kd) -> ndarray:
    global n

    # DATA COLLECTION INIT
    dt_list = []
    tick_time_list = []

    altitude_list = []
    altitude_error_list = []

    roc_list = []
    roc_error_list = []
    roc_dif_list = []
    target_roc_list = []
    target_roc_clipped_list = []

    current_pitch_list = []
    target_pitch_change_delta_list = []
    target_pitch_change_delta_clipped_list = []

    elevator_deflection_delta_list = []
    elevator_deflection_delta_clipped_list = []
    elevator_deflection_full_list = []
    elevator_deflection_full_clipped_list = []

    # SETTING START POSITION
    start_altitude = random.randint(2000, 4000)
    FGUtils.set_altitude(start_altitude)
    #  target_altitude = random.randint(2000, 4000)
    target_altitude = start_altitude + 300 # * random.choice([-1, 1])
    FGUtils.set_pitch(-2)
    FGUtils.set_heading_model(180)
    FGUtils.set_throttle(1)
    FGUtils.set_roll(0)
    FGUtils.set_rudder(0)
    FGUtils.set_elevator(0)
    FGUtils.set_aileron(0)

    # VARIABLE VALUES
    last_roc = FGUtils.get_vertical_speed()
    crash_coefficient = 1

    # STARTING CONTROLLER
    aircraft_controller = AircraftController(roc_kp, roc_ki, roc_kd, pitch_kp, pitch_ki, pitch_kd, elevator_kp,
                                             elevator_ki, elevator_kd)

    # TIMER
    start_time = time.time()
    allotted_time = (abs(target_altitude - start_altitude) / 5) * 1

    # DEBUG
    print(f"Allotted time: {allotted_time}")
    print(f"Start: {start_altitude} - Target: {target_altitude}")
    print(f"roc_kp: {roc_kp} roc_ki: {roc_ki} roc_kd: {roc_kd} pitch_kp: {pitch_kp} pitch_ki: {pitch_ki} pitch_kd: {pitch_kd} elevator_kp: {elevator_kp} elevator_ki: {elevator_ki} elevator_kd: {elevator_kd}")

    while time.time() - start_time < allotted_time:

        # AILERON | RUDDER P-CONTROLLER
        FGUtils.aileron_rudder_p_controller()

        # ALTITUDE CONTROLLER
        alt_ft = FGUtils.get_altitude_above_sea()
        climb_rate_ft_per_s = FGUtils.get_vertical_speed()
        current_elevator = FGUtils.get_elevator()
        current_pitch = FGUtils.get_pitch()
        elevator_signal = aircraft_controller.update(target_altitude, alt_ft, climb_rate_ft_per_s, current_pitch, current_elevator)
        FGUtils.set_elevator(elevator_signal)

        # DATA COLLECTION
        dt_list.append(aircraft_controller.dt)
        tick_time_list.append(aircraft_controller.last_time)

        altitude_list.append(alt_ft)
        #  altitude_error in metrics

        roc_list.append(climb_rate_ft_per_s)
        roc_error_list.append(aircraft_controller.roc_error)
        target_roc_list.append(aircraft_controller.target_roc)
        target_roc_clipped_list.append(aircraft_controller.target_roc_clipped)
        #  roc_diff in metrics

        current_pitch_list.append(current_pitch)
        target_pitch_change_delta_list.append(aircraft_controller.target_pitch_change_delta)
        target_pitch_change_delta_clipped_list.append(aircraft_controller.target_pitch_change_delta_clipped)

        elevator_deflection_delta_list.append(aircraft_controller.elevator_deflection_delta)
        elevator_deflection_delta_clipped_list.append(aircraft_controller.elevator_deflection_delta_clipped)
        elevator_deflection_full_list.append(aircraft_controller.elevator_deflection_full)
        elevator_deflection_full_clipped_list.append(elevator_signal)

        # METRICS
        altitude_error_list.append(aircraft_controller.altitude_error)

        roc_dif = climb_rate_ft_per_s - last_roc
        roc_dif_list.append(roc_dif)
        last_roc = climb_rate_ft_per_s

        # CRASH PREVENT
        if FGUtils.get_altitude_above_ground() < 200:
            crash_coefficient = 100000
            break

    # METRICS CALCULATION
    coefficient = max(20, abs(start_altitude - target_altitude))
    l1_altitude_error = ((np.sum(np.abs(np.array(altitude_error_list) / coefficient)) * crash_coefficient) / abs(start_altitude - target_altitude)) / allotted_time
    l1_vertical_speed_dif = ((np.sum(np.abs(np.array(roc_dif_list) / np.sqrt(coefficient))) * crash_coefficient) / allotted_time) / 50

    # SAVE LOGS
    data = [
        dt_list,
        tick_time_list,
        altitude_list,
        altitude_error_list,
        roc_list,
        roc_error_list,
        target_roc_list,
        target_roc_clipped_list,
        roc_dif_list,
        current_pitch_list,
        target_pitch_change_delta_list,
        target_pitch_change_delta_clipped_list,
        elevator_deflection_delta_list,
        elevator_deflection_delta_clipped_list,
        elevator_deflection_full_list,
        elevator_deflection_full_clipped_list,
    ]
    pid_logger(n, start_altitude, target_altitude, l1_altitude_error, l1_vertical_speed_dif, data)

    # DEBUG INFO
    print(f"L1 | altitude_error: {l1_altitude_error}\n"
          f"L1 | vertical_speed_dif: {l1_vertical_speed_dif}\n"
          f"L1_summ = {l1_altitude_error + l1_vertical_speed_dif}\n"
          f"Last_Altitude = {alt_ft}\n")
    n += 1
    return l1_altitude_error + l1_vertical_speed_dif


def objective(params):
    roc_kp, pitch_kp, elevator_kp = params
    return maneuvering(roc_kp, 1, 1, pitch_kp, 1, 1, elevator_kp, 1, 1)

def test():
    while True:
        print(FGUtils.get_pitch(), FGUtils.get_elevator(), FGUtils.get_vertical_speed())

if __name__ == "__main__":
    props_conn = PropsConnection('localhost', 5500)
    props_conn.connect()
    FGUtils = FGUtils(props_conn)

    space = [
        Real(-4, 1, name='roc_kp'),
        #Real(-2, 0, name='roc_ki'),
        #Real(-2, -1, name='roc_kd'),
        Real(-4, -1, name='pitch_kp'),
        #Real(-2, 0, name='pitch_ki'),
        #Real(-2, -1, name='pitch_kd'),
        Real(-4, -1, name='elevator_kp'),
        #Real(-2, 0, name='elevator_ki'),
        #Real(-2, -1, name='elevator_kd'),
    ]
    #test()
    maneuvering(-2.1, -1, 1, -2, -10, 1, -3, -10, 1)

    #result = gp_minimize(objective, space, n_calls=150, n_random_starts=1, x0=[-1, -2, -1])
    #print(f'Best parameters: {result.x}')
    #print(f'Best score: {result.fun}')
