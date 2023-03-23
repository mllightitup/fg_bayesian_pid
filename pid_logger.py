import numpy as np
import pandas as pd


def pid_logger(n, start_altitude, target_altitude, l1_altitude_error, l1_vertical_speed_dif, data_lists):
    filename = f"data/{n}_Start={start_altitude}_Target={target_altitude}_L1_A-E={round(l1_altitude_error, 3)}_L1-ROC-D={round(l1_vertical_speed_dif, 3)}"
    np.savetxt(f"{filename}.txt", np.column_stack(data_lists), fmt="%.8f", delimiter=",",
               header="dt,tick_time,altitude,altitude_error,roc,roc_error,target_roc,"
                      "target_roc_clipped,roc_dif,current_pitch_list,target_pitch_change_delta,"
                      "target_pitch_change_delta_clipped,elevator_deflection_delta,"
                      "elevator_deflection_delta_clipped,elevator_deflection_full,"
                      "elevator_deflection_full_clipped")

    data = pd.DataFrame({
        "dt": data_lists[0],
        'Tick Time': data_lists[1],
        'Altitude': data_lists[2],
        'Altitude Error': data_lists[3],
        'ROC': data_lists[4],
        'ROC Error': data_lists[5],
        'Target ROC': data_lists[6],
        'Target ROC Clipped': data_lists[7],
        'ROC Difference': data_lists[8],
        'Current Pitch': data_lists[9],
        'Target Pitch Change Delta': data_lists[10],
        'Target Pitch Change Delta Clipped': data_lists[11],
        'Elevator Deflection Delta': data_lists[12],
        'Elevator Deflection Delta Clipped': data_lists[13],
        'Elevator Deflection Full': data_lists[14],
        'Elevator Deflection Full Clipped': data_lists[15]
    })

    html_table = data.to_html()  # index=False
    filename = f"data/{n}_Start={start_altitude}_Target={target_altitude}_L1_A-E={round(l1_altitude_error, 3)}_L1-ROC-D={round(l1_vertical_speed_dif, 3)}"
    with open(f'{filename}.html', 'w') as f:
        f.write(html_table)
