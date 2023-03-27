from time import perf_counter
import numpy as np


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = 10 ** kp
        self.ki = 10 ** ki * self.kp
        self.kd = 10 ** kd * self.kp
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

MIN_AIRCRAFT_PITCH = -5
MAX_AIRCRAFT_PITCH = 5

MIN_AIRCRAFT_ELEVATOR_DEFLECTION = -0.005
MAX_AIRCRAFT_ELEVATOR_DEFLECTION = 0.22
AIRCRAFT_ELEVATOR_PERIOD = 2


class AircraftController:
    def __init__(self,
                 roc_kp: float, roc_ki: float, roc_kd: float,
                 elevator_kp: float, elevator_ki: float, elevator_kd: float):
        self.roc_controller = PIDController(roc_kp, roc_ki, roc_kd)
        self.elevator_controller = PIDController(elevator_kp, elevator_ki, elevator_kd)
        self.last_time = perf_counter()

        self.dt = None
        self.elevator_deflection_full_clipped = None
        self.elevator_deflection_full = None
        self.elevator_deflection_delta_clipped = None
        self.elevator_deflection_delta = None
        self.roc_error = None
        self.target_roc_clipped = None
        self.altitude_error = None
        self.target_roc = None
        self.target_pitch_change_delta = None

    def update(self, target_altitude, current_altitude, current_roc, current_pitch, current_elevator_deflection):
        current_time = perf_counter()
        self.dt = current_time - self.last_time
        self.last_time = current_time

        self.altitude_error = current_altitude - target_altitude

        self.target_roc = -self.roc_controller.update(self.altitude_error, self.dt)
        self.target_roc_clipped = np.clip(self.target_roc, -MAX_AIRCRAFT_ROC, MAX_AIRCRAFT_ROC)

        self.roc_error = current_roc - self.target_roc_clipped

        # servo_simulation = (2 / AIRCRAFT_ELEVATOR_PERIOD) * self.dt
        self.elevator_deflection_delta = self.elevator_controller.update(self.roc_error, self.dt)
        # self.elevator_deflection_delta = -self.elevator_controller.update(self.target_pitch_change_delta_clipped, self.dt)
        servo_simulation = 1
        self.elevator_deflection_delta_clipped = np.clip(self.elevator_deflection_delta, -servo_simulation,
                                                         servo_simulation)

        self.elevator_deflection_full = self.elevator_deflection_delta_clipped + current_elevator_deflection
        self.elevator_deflection_full_clipped = np.clip(self.elevator_deflection_full, MIN_AIRCRAFT_ELEVATOR_DEFLECTION,
                                                        MAX_AIRCRAFT_ELEVATOR_DEFLECTION)

        return self.elevator_deflection_full_clipped
