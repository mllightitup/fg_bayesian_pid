import random
import time

import numpy as np
from flightgear_python.fg_if import PropsConnection
from scipy import integrate
import matplotlib.pyplot as plt

from skopt import gp_minimize
from skopt.space import Real

from flightgear_utils_telnet import FGUtils

n = 1


def gauss_mf(x, mn, std, edge=None):
    # для чисел:
    if type(x) != np.ndarray:
        if edge == 'left':
            return np.exp(-((x - mn) / std) ** 2 / 2) if x > mn else 1
        elif edge == 'right':
            return np.exp(-((x - mn) / std) ** 2 / 2) if x < mn else 1

        return np.exp(-((x - mn) / std) ** 2 / 2)

        # для массивов:
    a = np.ones_like(x, dtype=np.float16)
    if edge == 'left':
        ind = np.where(x > mn)
        a[ind] = np.exp(-((x - mn) / std) ** 2 / 2)[ind]
        return a
    elif edge == 'right':
        ind = np.where(x < mn)
        a[ind] = np.exp(-((x - mn) / std) ** 2 / 2)[ind]
        return a

    return np.exp(-((x - mn) / std) ** 2 / 2)


def mf_draw(mn, std, edge=None):
    x = np.linspace(mn - 4 * std, mn + 4 * std, 50)
    y = gauss_mf(x, mn, std, edge)
    plt.plot(x, y)


def prm_to_log_space2(prm):
    """переводит массив 5х2 (mean,std для 5ти колокольчиков)
    в список 9ти параметров (in log space) на оптимизацию """
    log_prm = []

    m_2 = - prm[0, 0]
    m_1 = - prm[1, 0]
    m1 = prm[3, 0]
    m2 = prm[4, 0]
    s_2, s_1, s0, s1, s2 = prm[:, 1]

    s_2 = (m_2 - m_1) / s_2
    s_1 = max(m_2 - m_1, m_1) / s_1
    s0 = max(m_1, m1) / s0
    s1 = max(m2 - m1, m1) / s1
    s2 = (m2 - m1) / s2

    m_1 = m_1 / m_2
    m1 = m1 / m2

    # m_2 - логарифм середины крайнего левого колокольчика
    # m_1 - в диапазоне [log(0.2), log(0.8)]
    # m1 - в диапазоне [log(0.2), log(0.8)]
    # m2 - логарифм середины крайнего правого колокольчика
    # s_2, s_1, s0, s1, s2 - в диапазоне [log(2), log(4)]
    return np.log10(np.array([m_2, m_1, m1, m2, s_2, s_1, s0, s1, s2]))


def log_prm_to_lin_space2(variable_type, p0=0, m=0, p1=0, p2=0):
    """inverse of prm_to_log_space2()"""
    if variable_type == "Altitude":
        lm_2 = np.log10(50.0)
        lm_1 = np.log10(0.5)
        lm1 = np.log10(0.5)
        lm2 = np.log10(50.0)
        ls0, ls_1, ls_2, ls1, ls2 = p0, p0, p0, p0, p0

    if variable_type == "ROC":
        lm_2 = np.log10(5)
        lm_1 = np.log10(0.5)
        lm1 = np.log10(0.5)
        lm2 = np.log10(5.0)
        ls0, ls_1, ls_2, ls1, ls2 = p0, p0, p0, p0, p0

    if variable_type == "Elevator":
        lm_2 = m
        lm_1 = np.log10(0.5)
        lm1 = np.log10(0.5)
        lm2 = m
        ls0, ls_1, ls_2, ls1, ls2 = p0, p1, p2, p1, p2

    lin_prm = np.zeros((5, 2))

    lin_prm[0, 0] = - 10 ** lm_2
    lin_prm[1, 0] = lin_prm[0, 0] * 10 ** lm_1
    lin_prm[2, 0] = 0
    lin_prm[4, 0] = 10 ** lm2
    lin_prm[3, 0] = lin_prm[4, 0] * 10 ** lm1

    lin_prm[0, 1] = (lin_prm[1, 0] - lin_prm[0, 0]) / 10 ** ls_2
    lin_prm[1, 1] = max(lin_prm[1, 0] - lin_prm[0, 0], -lin_prm[1, 0]) / 10 ** ls_1
    lin_prm[2, 1] = max(-lin_prm[1, 0], lin_prm[3, 0]) / 10 ** ls0
    lin_prm[3, 1] = max(lin_prm[4, 0] - lin_prm[3, 0], lin_prm[3, 0]) / 10 ** ls1
    lin_prm[4, 1] = (lin_prm[4, 0] - lin_prm[3, 0]) / 10 ** ls2

    return lin_prm


rules = [
    {'droc': -2, 'accel': [-2, -1, 0], 'elevator': -2},
    {'droc': -2, 'accel': [1], 'elevator': -1},
    {'droc': -2, 'accel': [2], 'elevator': 0},  # 0?
    {'droc': -1, 'accel': [-2, -1], 'elevator': -2},
    {'droc': -1, 'accel': [0], 'elevator': -1},
    {'droc': -1, 'accel': [1], 'elevator': 0},
    {'droc': -1, 'accel': [2], 'elevator': 1},
    {'droc': 0, 'accel': [-2], 'elevator': -2},
    {'droc': 0, 'accel': [-1], 'elevator': -1},
    {'droc': 0, 'accel': [0], 'elevator': 0},
    {'droc': 0, 'accel': [1], 'elevator': 1},
    {'droc': 0, 'accel': [2], 'elevator': 2},
    {'droc': 1, 'accel': [-2], 'elevator': -1},
    {'droc': 1, 'accel': [-1], 'elevator': 0},
    {'droc': 1, 'accel': [0], 'elevator': 1},
    {'droc': 1, 'accel': [1, 2], 'elevator': 2},
    {'droc': 2, 'accel': [-2], 'elevator': 0},  # 0
    {'droc': 2, 'accel': [-1], 'elevator': 1},
    {'droc': 2, 'accel': [0, 1, 2], 'elevator': 2},
]


class Elevator:
    def __init__(self, droc_linprm, accel_linprm, elevator_change_linprm, vmax, h_th):

        self.droc_linprm = droc_linprm
        self.accel_linprm = accel_linprm
        self.elevator_change_linprm = elevator_change_linprm
        self.vmax = vmax
        self.h_th = h_th

    def get_target_roc(self, dh):
        sgn = - 1 if dh > 0 else 1
        target_roc = self.vmax * sgn
        if -dh * sgn < self.h_th:
            target_roc = self.vmax / self.h_th ** 2 * dh ** 2 * sgn
        return target_roc

    def update(self, dh, roc, accel):
        # applies rules to current state
        self.defuzz_shapes = []  # [(mn, std для колокольчика + планка уверенности)]
        self.a = 1  # нижняя граница интегрирования для последующей итерации
        self.b = -1  # верхняя граница интегрирования

        droc = roc - self.get_target_roc(dh)
        if accel == 0:
            print('roc', roc, 'target_roc', roc - droc, 'droc', droc)

        for rule in rules:
            # определение фигуры дефузификации, по которой будет рассчитываться центр масс
            [mn, std] = self.droc_linprm[rule['droc'] + 2]
            # print(altitude_error_linprm[rule['h'] + 2])
            droc_mf = gauss_mf(droc, mn, std,
                               edge='left' if rule['droc'] == -2 else 'right' if rule['droc'] == 2 else None)

            accel_mf = 0
            for rule_accel in rule['accel']:
                [mn, std] = self.accel_linprm[rule_accel + 2]
                # print(roc, mn, std)
                x = gauss_mf(accel, mn, std, edge='left' if rule_accel == -2 else 'right' if rule_accel == 2 else None)
                # print('x', x)
                if x > accel_mf:
                    accel_mf = x
                    # print('roc_mf', roc_mf)

            mf_value = min(droc_mf, accel_mf)

            if mf_value > 5e-4:  # std < 3.9 sigma
                [mn, std] = self.elevator_change_linprm[rule['elevator'] + 2]
                # if accel==0:
                #  print(droc, droc_mf, accel_mf, mf_value, 'elevator',[mn, std])
                self.defuzz_shapes.append((mn, std, mf_value))

                if mn - 3 * std < self.a:
                    self.a = mn - 3 * std

                if mn + 3 * std > self.b:
                    self.b = mn + 3 * std

        return self

    def calc_point_mf(self, x):
        # значение фигуры дефузи в точке
        y = 0
        for mn, std, mf_value in self.defuzz_shapes:
            yt = min(gauss_mf(x, mn, std), mf_value)
            if yt > y:
                y = yt
        return y

    def calc_point_xmf(self, x):
        # значение фигуры дефузи в точке * x
        y = 0
        for mn, std, mf_value in self.defuzz_shapes:
            yt = min(gauss_mf(x, mn, std), mf_value)
            if yt > y:
                y = yt
        return y * x

    def get_cm(self):
        # return: х-координата центра масс фигуры дефузи
        xx = integrate.quad(self.calc_point_xmf, self.a, self.b, epsrel=5e-2)
        x = integrate.quad(self.calc_point_mf, self.a, self.b, epsrel=5e-2)

        return xx[0] / x[0] if x[0] != 0 else 0


# a = np.log10(2)
# elevator_change_linprm = log_prm_to_lin_space2("Elevator" , m=-1, p0=a, p1=a, p2=a)
# altitude_error_linprm = log_prm_to_lin_space2("Altitude" , p0=a)
# current_roc_linprm = log_prm_to_lin_space2("ROC" , m=a)
# elevator_controller = Elevator(elevator_change_linprm, altitude_error_linprm, current_roc_linprm)
# for i in
#     elevator_controller.update(-50, 1)
#     elevator_change_signal = elevator_controller.get_cm()
# print(elevator_change_signal)
def pid_logger(n, start_altitude, target_altitude, l1_altitude_error, l1_vertical_speed_dif, data,
               roc_dif_list, elevator_change_linprm, altitude_error_linprm, current_roc_linprm):
    filename = f"data/{n}_Start={start_altitude}_Target={target_altitude}_L1_A-E={l1_altitude_error}_L1-ROC-D={l1_vertical_speed_dif}"

    with open(f'data/{n}_metadata.txt', 'w') as f:
        f.write(
            f"{start_altitude} {target_altitude} {l1_altitude_error} {l1_vertical_speed_dif} \n{elevator_change_linprm}\n{altitude_error_linprm}\n{current_roc_linprm}")

    np.savetxt(f'{filename}.csv', np.column_stack((data, roc_dif_list)), delimiter=',', fmt="%.8f",
               header="current_altitude,dh,current_roc,roc,current_pitch,"
                      "elevator_change_signal,elevator_change_full,roc_dif")


def maneuvering(elevator_change_linprm, droc_linprm, accel_linprm):
    global n

    # SETTING START POSITION
    start_altitude = random.randint(2000, 3000)
    FGUtils.set_altitude(start_altitude)
    #FGUtils.set_heading_model(180)
    FGUtils.set_throttle(1)
    #FGUtils.set_roll(0)
    #FGUtils.set_rudder(0)
    #FGUtils.set_elevator(0)
    #FGUtils.set_aileron(0)

    # TARGETS
    target_altitude = start_altitude + 1000
    target_roc = 5

    # TIMER
    start_time = time.time()
    allotted_time = (abs(target_altitude - start_altitude) / 5) * 2
    print(allotted_time)

    # DATA COLLECTION INIT
    i = 0
    array_max_size = int(allotted_time / 0.15)
    print(array_max_size)
    data = np.zeros((array_max_size, 5))
    altitude_error_list = np.zeros(array_max_size)
    roc_dif_list = np.zeros(array_max_size)

    # VARIABLE VALUES
    crash_coefficient = 1
    last_roc = FGUtils.get_vertical_speed()
    last_dt = time.perf_counter()

    # STARTING CONTROLLER
    elevator_controller = Elevator(droc_linprm, accel_linprm, elevator_change_linprm, vmax=20, h_th=50)

    while time.time() - start_time < allotted_time:
        # AILERON | RUDDER P-CONTROLLER
        FGUtils.aileron_rudder_p_controller()

        # ALTITUDE CONTROLLER
        current_altitude = FGUtils.get_altitude_above_sea()
        dh = current_altitude - target_altitude

        current_roc = FGUtils.get_vertical_speed()
        # roc = current_roc - target_roc
        current_time = time.perf_counter()
        dt = current_time - last_dt
        last_dt = current_time
        print(f"{current_altitude} - {current_roc}")
        current_elevator = FGUtils.get_elevator()
        roc_dif = current_roc - last_roc
        accel = roc_dif / dt
        elevator_controller.update(dh, current_roc, accel)
        elevator_change_signal = elevator_controller.get_cm()
        elevator_change_full = max(0, min(elevator_change_signal + current_elevator, 1))
        FGUtils.set_elevator(elevator_change_full)

        # METRICS
        altitude_error_list[i] = dh

        roc_dif_list[i] = roc_dif
        last_roc = current_roc

        # DATA COLLECTION
        data[i, :] = [
            current_altitude,
            dh,
            current_roc,
            # roc,
            # current_pitch,
            elevator_change_signal,
            elevator_change_full,
        ]

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
    pid_logger(n, start_altitude, target_altitude, l1_altitude_error, l1_vertical_speed_dif, data,
               roc_dif_list, elevator_change_linprm, droc_linprm, accel_linprm)
    n += 1
    return l1_altitude_error + l1_vertical_speed_dif


def objective(params):
    elevator_change_linprm = log_prm_to_lin_space2("Elevator", m=params[0], p0=params[1], p1=params[2], p2=params[3])
    altitude_error_linprm = log_prm_to_lin_space2("Altitude", p0=params[4])
    current_roc_linprm = log_prm_to_lin_space2("ROC", p0=params[5])
    return maneuvering(elevator_change_linprm, altitude_error_linprm, current_roc_linprm)


if __name__ == "__main__":
    props_conn = PropsConnection('localhost', 5500)
    props_conn.connect()
    FGUtils = FGUtils(props_conn)

    # m_2 - логарифм середины крайнего левого колокольчика
    # m_1 - в диапазоне [log(0.2), log(0.8)]
    # m1 - в диапазоне [log(0.2), log(0.8)]
    # m2 - логарифм середины крайнего правого колокольчика
    # s_2, s_1, s0, s1, s2 - в диапазоне [log(2), log(4)]
    a = np.log10(1.8)
    b = np.log10(2.2)
    space = [
        # Real(-5, -3, name='elevator_m_2'),
        # Real(np.log10(0.2), np.log10(0.8), name='elevator_m_1'),
        # Real(np.log10(0.2), np.log10(0.8), name='elevator_m1'),
        Real(-3, -1, name='elevator_m2'),
        # Real(np.log10(2), np.log10(4), name='elevator_s_2'),
        # Real(np.log10(2), np.log10(4), name='elevator_s_1'),
        Real(a, b, name='elevator_s0'),
        Real(a, b, name='elevator_s1'),
        Real(a, b, name='elevator_s2'),
        # Real(0, 2, name='alt_m_2'),
        # Real(np.log10(0.2), np.log10(0.8), name='alt_m_1'),
        # Real(np.log10(0.2), np.log10(0.8), name='alt_m1'),
        # Real(0, 2, name='alt_m2'),
        # Real(np.log10(2), np.log10(4), name='alt_s_2'),
        # Real(np.log10(2), np.log10(4), name='alt_s_1'),
        # Real(np.log10(2), np.log10(4), name='alt_s0'),
        # Real(np.log10(2), np.log10(4), name='alt_s1'),
        Real(a, b, name='alt_s2'),
        # Real(-1, 1, name='roc_m_2'),
        # Real(np.log10(0.2), np.log10(0.8), name='roc_m_1'),
        # Real(np.log10(0.2), np.log10(0.8), name='roc_m1'),
        # Real(-1, 1, name='roc_m2'),
        # Real(np.log10(2), np.log10(4), name='roc_s_2'),
        # Real(np.log10(2), np.log10(4), name='roc_s_1'),
        # Real(np.log10(2), np.log10(4), name='roc_s0'),
        # Real(np.log10(2), np.log10(4), name='roc_s1'),
        Real(a, b, name='roc_s2'),
    ]
    maneuvering(np.array([[-0.0696, 0.013572],
                          [-0.045936, 0.013572],
                          [0., 0.013572],
                          [0.045936, 0.013572],
                          [0.0696, 0.013572]]),
                np.array([[-232., 42.34],
                          [-69.6, 42.34],
                          [0., 42.34],
                          [69.6, 42.34],
                          [232., 42.34]]),
                np.array([[-294.64, 14.732],
                          [-26.5176, 14.732],
                          [0., 14.732],
                          [26.5176, 14.732],
                          [294.64, 14.732]]))
    # result = gp_minimize(objective, space, n_calls=50, n_random_starts=1)
    # print(f'Best parameters: {result.x}')
    # print(f'Best score: {result.fun}')
