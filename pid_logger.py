import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def pid_logger(n, start_altitude, target_altitude, l1_altitude_error, l1_vertical_speed_dif, data, altitude_error_list,
               roc_dif_list):
    filename = f"data/{n}_Start={start_altitude}_Target={target_altitude}_L1_A-E={l1_altitude_error}_L1-ROC-D={l1_vertical_speed_dif}"

    np.savetxt(f'{filename}.csv', np.column_stack((data, altitude_error_list, roc_dif_list)), delimiter=',', fmt="%.8f",
               header="dt,tick_time,altitude,roc,roc_error,target_roc,"
                      "target_roc_clipped,current_pitch,elevator_deflection_delta,"
                      "elevator_deflection_delta_clipped,elevator_deflection_full,"
                      "elevator_deflection_full_clipped,altitude_error,roc_dif")


def create_aircraft_data_plot(data, altitude_error_list, roc_dif_list, allotted_time, start_altitude, target_altitude,
                              roc_kp, roc_ki, roc_kd, elevator_kp, elevator_ki, elevator_kd, l1_altitude_error,
                              l1_vertical_speed_dif):
    fig, axs = plt.subplots(nrows=7, ncols=2, figsize=(12, 18))

    title = f'Aircraft data\n' \
            f'Allotted time: {allotted_time}\nStart: {start_altitude} - Target: {target_altitude}\n' \
            f'roc_kp: {roc_kp} roc_ki: {roc_ki} roc_kd: {roc_kd}\n' \
            f'elevator_kp: {elevator_kp} elevator_ki: {elevator_ki} elevator_kd: {elevator_kd}\n' \
            f'L1 | altitude_error: {l1_altitude_error}\nL1 | vertical_speed_dif: {l1_vertical_speed_dif}'
    fig.suptitle(title)

    subplot_titles = ['dt', 'tick_time', 'altitude', 'roc', 'roc_error', 'target_roc', 'target_roc_clipped',
                      'current_pitch', 'elevator_deflection_delta', 'elevator_deflection_delta_clipped',
                      'elevator_deflection_full', 'elevator_deflection_full_clipped', 'altitude_error', 'roc_dif']

    for i, title in enumerate(subplot_titles):
        row = i // 2
        col = i % 2
        if i < 12:
            axs[row, col].plot(data[:, i])
        elif i == 12:
            axs[row, col].plot(altitude_error_list)
        else:
            axs[row, col].plot(roc_dif_list)
        axs[row, col].set_title(title)
        axs[row, col].set(xlabel='Time', ylabel='Value')
        axs[row, col].set_xlim([0, allotted_time / 0.3])  # add this line to limit x-axis to 100

    plt.tight_layout()
    plt.show()
