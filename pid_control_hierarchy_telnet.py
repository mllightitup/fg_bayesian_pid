from time import perf_counter
import numpy as np


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = 10 ** kp
        self.ki = 10 ** ki
        self.kd = 10 ** kd
        self.last_error = None
        self.integral_error = 0

    def update(self, error: float, dt: float) -> float:
        self.integral_error += error * dt
        derivative_error = (error - self.last_error) / dt if self.last_error is not None else 0
        self.last_error = error

        return (
                error * self.kp
                + self.integral_error * self.ki
                + derivative_error * self.kd
        )


MAX_AIRCRAFT_ROC = 5

MIN_AIRCRAFT_PITCH = -2
MAX_AIRCRAFT_PITCH = 2

MIN_AIRCRAFT_ELEVATOR_DEFLECTION = -0.1
MAX_AIRCRAFT_ELEVATOR_DEFLECTION = 0.3
AIRCRAFT_ELEVATOR_PERIOD = 2


class AircraftController:
    def __init__(self,
                 roc_kp: float, roc_ki: float, roc_kd: float,
                 pitch_kp: float, pitch_ki: float, pitch_kd: float,
                 elevator_kp: float, elevator_ki: float, elevator_kd: float):
        self.target_roc = None
        self.roc_controller = PIDController(roc_kp, roc_ki, roc_kd)
        self.pitch_controller = PIDController(pitch_kp, pitch_ki, pitch_kd)
        self.elevator_controller = PIDController(elevator_kp, elevator_ki, elevator_kd)
        self.last_time = perf_counter()

    def update(self, target_altitude, current_altitude, current_roc, current_elevator_deflection):
        current_time = perf_counter()
        dt = current_time - self.last_time
        self.last_time = current_time

        altitude_error = current_altitude - target_altitude
        self.target_roc = -self.roc_controller.update(altitude_error, dt)
        target_roc_clipped = np.clip(self.target_roc, -MAX_AIRCRAFT_ROC, MAX_AIRCRAFT_ROC)

        roc_error = current_roc - target_roc_clipped
        target_pitch_change_delta = -self.pitch_controller.update(roc_error, dt)
        target_pitch_change_delta_clipped = np.clip(target_pitch_change_delta, MIN_AIRCRAFT_PITCH, MAX_AIRCRAFT_PITCH)

        servo_simulation = (2 / AIRCRAFT_ELEVATOR_PERIOD) * dt
        elevator_deflection_delta = -self.elevator_controller.update(target_pitch_change_delta_clipped, dt)
        elevator_deflection_delta_clipped = np.clip(elevator_deflection_delta, -servo_simulation, servo_simulation)

        elevator_deflection_full = elevator_deflection_delta_clipped + current_elevator_deflection
        elevator_deflection_full_clipped = np.clip(elevator_deflection_full, MIN_AIRCRAFT_ELEVATOR_DEFLECTION,
                                                   MAX_AIRCRAFT_ELEVATOR_DEFLECTION)

        # print(f"altitude: {current_altitude} t_altitude: {target_altitude} t_roc: {self.target_roc}\n"
        #       f"roc: {current_roc} t_roc: {target_roc_clipped} pitch_delta: {target_pitch_change_delta}\n"
        #       f"e_delta: {elevator_deflection_delta} e_delta_clipped: {elevator_deflection_delta_clipped}\n"
        #       f"e_full: {elevator_deflection_full} e_current: {current_elevator_deflection}\n")
        return elevator_deflection_full_clipped
