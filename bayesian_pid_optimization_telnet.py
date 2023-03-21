import datetime
import random
import time

from flightgear_python.fg_if import PropsConnection
from numpy import ndarray
import numpy as np
from flightgear_utils_telnet import FGUtils
from pid_control_hierarchy_telnet import AircraftController
from skopt import gp_minimize
from skopt.space import Real

n = 1


def maneuvering(roc_kp, roc_ki, roc_kd, pitch_kp, pitch_ki, pitch_kd, elevator_kp, elevator_ki, elevator_kd) -> ndarray:
    global n
    # Создаем экземпляры классов
    aircraft_controller = AircraftController(roc_kp, roc_ki, roc_kd, pitch_kp, pitch_ki, pitch_kd, elevator_kp,
                                             elevator_ki, elevator_kd)

    start_altitude = random.randint(2000, 4000)
    FGUtils.set_altitude(start_altitude)
    target_altitude = random.randint(2000, 4000)
    FGUtils.set_vertical_speed(0)
    FGUtils.set_ground_speed(110)
    FGUtils.set_roll(0)
    FGUtils.set_pitch(0)
    FGUtils.set_heading_model(180)
    FGUtils.set_throttle(1)

    last_roc = FGUtils.get_vertical_speed()
    crash_coefficient = 1
    roll_deg_setpoint = 0
    head_deg_setpoint = 180
    roll_Kp = 0.01
    head_Kp = 0.01

    # DATA COLLECTION INIT
    altitude_error_list = []
    roc_dif_list = []
    altitude_list = []
    roc_list = []
    elevator_deflection_list = []
    target_roc_list = []
    tick_time_list = []

    print(f"Start: {start_altitude} - Target: {target_altitude}")
    print(f"roc_kp: {roc_kp} roc_ki: {roc_ki} roc_kd: {roc_kd}\n"
          f"pitch_kp: {pitch_kp} pitch_ki: {pitch_ki} pitch_kd: {pitch_kd}\n"
          f"elevator_kp: {elevator_kp} elevator_ki: {elevator_ki} elevator_kd: {elevator_kd}")

    start_time = time.time()
    while time.time() - start_time < 180:

        # AILERON | RUDDER P-CONTROLLER
        roll_error = roll_deg_setpoint - FGUtils.get_roll_deg()
        head_error = head_deg_setpoint - FGUtils.get_heading()
        aileron = roll_error * roll_Kp
        rudder = max(min(head_error * head_Kp, 0.3), -0.3)
        FGUtils.set_rudder(rudder)
        FGUtils.set_aileron(aileron)

        # ALTITUDE CONTROLLER
        alt_ft = FGUtils.get_altitude_above_sea()
        climb_rate_ft_per_s = FGUtils.get_vertical_speed()
        current_elevator = FGUtils.get_elevator()
        elevator_signal = aircraft_controller.update(target_altitude, alt_ft, climb_rate_ft_per_s, current_elevator)
        FGUtils.set_elevator(elevator_signal)

        # DATA COLLECTION
        altitude_list.append(alt_ft)
        roc_list.append(climb_rate_ft_per_s)
        elevator_deflection_list.append(elevator_signal)
        target_roc_list.append(aircraft_controller.target_roc) #FIXME
        tick_time_list.append(aircraft_controller.last_time)

        # METRICS
        altitude_error = alt_ft - target_altitude
        altitude_error_list.append(altitude_error)

        roc_dif = climb_rate_ft_per_s - last_roc
        roc_dif_list.append(roc_dif)
        last_roc = climb_rate_ft_per_s

        # CRASH PREVENT
        if FGUtils.get_altitude_above_ground() < 150:
            crash_coefficient = 100000
            break
    # METRICS CALCULATION
    coefficient = max(20, abs(start_altitude - target_altitude))
    l1_altitude_error = np.sum(np.abs(np.array(altitude_error_list) / coefficient)) * crash_coefficient
    l1_vertical_speed_dif = np.sum(np.abs(np.array(roc_dif_list) / np.sqrt(coefficient))) * crash_coefficient

    # SAVE LOGS
    filename = f"data/{n}-Start={start_altitude}-Target={target_altitude}-L1_AE={round(l1_altitude_error, 3)}-L1_VSD={round(l1_vertical_speed_dif, 3)}.txt"
    data = [tick_time_list, altitude_list, roc_list, target_roc_list, altitude_error_list,
            roc_dif_list, elevator_deflection_list]
    np.savetxt(filename, np.column_stack(data), fmt="%.8f", delimiter=",",
               header="Tick_Time, Altitude, Current_ROC, Target_ROC, AltitudeError, ROC_Difference, Elevator_Deflection")

    # DEBUG INFO
    print(f"L1 | altitude_error: {l1_altitude_error}")
    print(f"L1 | vertical_speed_dif: {l1_vertical_speed_dif}\n")
    n += 1
    return l1_altitude_error + l1_vertical_speed_dif


def objective(params):
    roc_kp, roc_ki, roc_kd, pitch_kp, pitch_ki, pitch_kd, elevator_kp, elevator_ki, elevator_kd = params
    return maneuvering(roc_kp, roc_ki, roc_kd, pitch_kp, pitch_ki, pitch_kd, elevator_kp, elevator_ki, elevator_kd)


if __name__ == "__main__":
    props_conn = PropsConnection('localhost', 5500)
    props_conn.connect()
    FGUtils = FGUtils(props_conn)

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

    # maneuvering()

    result = gp_minimize(objective, space, n_calls=150, n_random_starts=30)
    print(f'Best parameters: {result.x}')
    print(f'Best score: {result.fun}')
