import random
import time

from pid_control_hierarchy_telnet import AircraftController
from pid_logger import pid_logger, create_aircraft_data_plot
from flightgear_utils_telnet import FGUtils
from flightgear_python.fg_if import PropsConnection

from numpy import ndarray
import numpy as np

from skopt import gp_minimize
from skopt.space import Real

n = 1


def maneuvering(roc_kp, roc_ki, roc_kd, elevator_kp, elevator_ki, elevator_kd) -> ndarray:
    global n

    # SETTING START POSITION
    start_altitude = random.randint(2000, 3000)
    FGUtils.set_altitude(start_altitude)
    target_altitude = start_altitude + 1000
    FGUtils.set_heading_model(180)
    FGUtils.set_throttle(1)
    FGUtils.set_roll(0)
    FGUtils.set_rudder(0)
    FGUtils.set_elevator(0)
    FGUtils.set_aileron(0)

    allotted_time = (abs(target_altitude - start_altitude) / 5) * 2

    # DATA COLLECTION INIT
    array_max_size = int(allotted_time/0.3)
    data = np.zeros((array_max_size, 13))
    altitude_error_list = np.zeros(array_max_size)
    roc_dif_list = np.zeros(array_max_size)
    i = 0

    # VARIABLE VALUES
    last_roc = FGUtils.get_vertical_speed()
    crash_coefficient = 1

    # STARTING CONTROLLER
    aircraft_controller = AircraftController(roc_kp, roc_ki, roc_kd, elevator_kp,
                                             elevator_ki, elevator_kd)

    # TIMER
    start_time = time.time()

    while time.time() - start_time < allotted_time:

        # AILERON | RUDDER P-CONTROLLER
        FGUtils.aileron_rudder_p_controller()

        # ALTITUDE CONTROLLER
        alt_ft = FGUtils.get_altitude_above_sea()
        climb_rate_ft_per_s = FGUtils.get_vertical_speed()
        print(FGUtils.get_vertical_speed_jsbsim(), climb_rate_ft_per_s, alt_ft)
        current_elevator = FGUtils.get_elevator()
        current_pitch = FGUtils.get_pitch()
        elevator_signal = aircraft_controller.update(target_altitude, alt_ft, climb_rate_ft_per_s, current_pitch,
                                                     current_elevator)
        FGUtils.set_elevator(elevator_signal)

        # DATA COLLECTION
        data[i, :] = [
            aircraft_controller.dt,
            aircraft_controller.last_time,
            alt_ft, climb_rate_ft_per_s,
            aircraft_controller.roc_error,
            aircraft_controller.target_roc,
            aircraft_controller.target_roc_clipped,
            current_pitch,
            climb_rate_ft_per_s,
            aircraft_controller.elevator_deflection_delta,
            aircraft_controller.elevator_deflection_delta_clipped,
            aircraft_controller.elevator_deflection_full,
            elevator_signal,
        ]

        # METRICS
        altitude_error_list[i] = aircraft_controller.altitude_error

        roc_dif = climb_rate_ft_per_s - last_roc
        roc_dif_list[i] = roc_dif
        last_roc = climb_rate_ft_per_s

        i += 1

        # CRASH PREVENT
        if FGUtils.get_altitude_above_ground() < 200:
            crash_coefficient = 10
            break

    # METRICS CALCULATION
    coefficient = max(20, abs(start_altitude - target_altitude))
    l1_altitude_error = ((np.sum(np.abs(np.array(altitude_error_list) / coefficient)) * crash_coefficient) / abs(
        start_altitude - target_altitude)) / allotted_time
    l1_vertical_speed_dif = ((np.sum(
        np.abs(np.array(roc_dif_list) / np.sqrt(coefficient))) * crash_coefficient) / allotted_time) / 5

    # SAVE LOGS
    pid_logger(n, start_altitude, target_altitude, l1_altitude_error, l1_vertical_speed_dif, data, altitude_error_list,
               roc_dif_list)
    create_aircraft_data_plot(data, altitude_error_list, roc_dif_list, allotted_time, start_altitude, target_altitude,
                              roc_kp, roc_ki, roc_kd, elevator_kp, elevator_ki, elevator_kd, l1_altitude_error,
                              l1_vertical_speed_dif)

    n += 1
    return l1_altitude_error + l1_vertical_speed_dif


def objective(params):
    roc_kp, roc_ki, roc_kd, elevator_kp, elevator_ki, elevator_kd = params
    return maneuvering(roc_kp, roc_ki, roc_kd, elevator_kp, elevator_ki, elevator_kd)


if __name__ == "__main__":
    props_conn = PropsConnection('localhost', 5500)
    props_conn.connect()
    FGUtils = FGUtils(props_conn)

    space = [
        Real(-2, -1, name='roc_kp'),
        Real(-4, -2, name='roc_ki'),
        Real(-5, -2, name='roc_kd'),
        Real(-5, -3, name='elevator_kp'),
        Real(-2, -0, name='elevator_ki'),
        Real(-2, 2, name='elevator_kd'),
    ]

    x0 = [
        [-1.1323897131484202, -3.0, -2.4457776352073366, -3.8181350976120267, -1.9376833562741715, 1.0],
        [-1.484395677075241, -2, -2, -4.0, -0.574321841003455, 0.7317159572934067],
        [-1.2411696011182358, -2.6398858942439016, -3.0, -3.595137025655778, -1.3641227060853973, 0.6627174644738796],
    ]
    result = gp_minimize(objective, space, n_calls=150, n_random_starts=0, x0=x0)
    print(f'Best parameters: {result.x}')
    print(f'Best score: {result.fun}')
