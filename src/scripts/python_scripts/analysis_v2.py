import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
import prettytable as ptb
import os
import sys
import copy
import math
import scipy.signal as ss


def analysis_v2(data_dir="/data", result_dir="/result"):
    # Get file paths
    current_dir = os.path.dirname(os.path.abspath(__file__))

    input_dir = current_dir + data_dir  # + "/history" # add "/history"
    if not os.path.exists(input_dir):
        print("There is no input directory. ")
        return

    output_dir = current_dir + result_dir
    if not os.path.exists(output_dir):
        print("There is no directory to store result pictures. ")
        return

    # Check if input directory is invalid
    csv_file_list = os.listdir(input_dir)
    if csv_file_list == []:
        print("No files found in input directory. ")
        return

    # Read csv files & process
    for current_file in csv_file_list:
        # Ignore invalid files
        if not current_file[-4:] == ".csv":
            continue

        if current_file[:13] == "vehicle_state":
            analysis_vehicle_state(current_file, input_dir, output_dir)

        if current_file[:10] == "trajectory":
            analysis_trajectory(current_file, input_dir, output_dir)


def analysis_vehicle_state(file_name, read_path, save_path):
    print("Analyzing vehicle_state file :" + file_name)
    current_file = file_name
    input_dir = read_path
    output_dir = save_path
    res_dict = {}

    # Init some dicts to identify data
    data = pd.read_csv(input_dir + "/" + current_file)
    headers_list = data.columns.values
    count = 0

    data = data.values
    for header in headers_list:
        res_dict[header.strip()] = data[:, count]
        count += 1

# ['timestamp_sec' ' raw_x' ' raw_y' ' raw_z' ' raw_roll' ' raw_pitch' ' raw_yaw' ' raw_heading'
#  ' curvature' ' angular_velocity' ' speed_enu_x' ' speed_enu_y'
#  ' speed_enu_z' ' localization_velocity' ' linear_velocity'
#  ' linear_acceleration' ' steering_angle' ' steering_angle_spd'
#  ' steering_percentage' ' gear' ' driving_mode' ' driving_mode_cmd'
#  ' gear_cmd' ' acceleration_cmd' ' steering_cmd' ' steering_rate_cmd'
#  ' driving_action' ' station_error' ' speed_error' ' lateral_error'
#  ' heading_error' ' control_iteration_cost_ms' ' localization_jump_count'
#  ' count_bad_kappa' ' count_bad_dkappa' ' diag_result_index'
#  ' fallback_reaction_index']

    # switch plot or not
    enable_plot = 1
    # plot_input = input("Do you want to plot pictures, please type y/n!")
    # if plot_input == 'y':
    #     enable_plot = 1

    # data process1
    # find start (time not 0)
    vehiclestate_total_length = len(res_dict['raw_x'])
    last_zero_index = -1
    for i in range(0, vehiclestate_total_length):
        if res_dict['timestamp_sec'][i] > 0.0 and abs(res_dict['raw_x'][i]) > 0.0:
            break
        else:
            last_zero_index = last_zero_index + 1
    vehiclestate_valid_start_index = last_zero_index + 1
    # find end (time not 0)
    vehiclestate_valid_end_index = vehiclestate_total_length - 1
    print(vehiclestate_valid_start_index, vehiclestate_valid_end_index)

    while vehiclestate_valid_end_index > vehiclestate_valid_start_index:
        if res_dict['timestamp_sec'][vehiclestate_valid_end_index] > res_dict['timestamp_sec'][vehiclestate_valid_end_index-1]:
            break
        vehiclestate_valid_end_index -= 1

    vehiclestate_valid_length = len(
        res_dict['timestamp_sec'][vehiclestate_valid_start_index:vehiclestate_valid_end_index])
    if vehiclestate_valid_length < 1:
        print("vehiclestate_valid_start_index", vehiclestate_valid_start_index)
        print("vehiclestate_valid_end_index", vehiclestate_valid_end_index)
        print("WARN:valid length == 0,no valid data!")
        print("return")
        return
    vehiclestate_timestamp = res_dict["timestamp_sec"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_x = res_dict["raw_x"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_y = res_dict["raw_y"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_z = res_dict["raw_z"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_roll = res_dict["raw_roll"][vehiclestate_valid_start_index:]
    vehiclestate_pitch = res_dict["raw_pitch"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_yaw = res_dict["raw_yaw"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_heading = res_dict["raw_heading"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_curvature = res_dict["curvature"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_angular_velocity = res_dict["angular_velocity"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_speed_x = res_dict["speed_enu_x"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_speed_y = res_dict["speed_enu_y"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_speed_z = res_dict["speed_enu_z"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_localization_velocity = res_dict["localization_velocity"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_linear_velocity = res_dict["linear_velocity"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_linear_acceleration = res_dict["linear_acceleration"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_steering_angle = res_dict["steering_angle"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_steering_angle_spd = res_dict["steering_angle_spd"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_steering_percentage = res_dict["steering_percentage"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_gear = res_dict["gear"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_driving_mode = res_dict["driving_mode"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_driving_mode_cmd = res_dict["driving_mode_cmd"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_gear_cmd = res_dict["gear_cmd"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_acceleration_cmd = res_dict["acceleration_cmd"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_steering_cmd = res_dict["steering_cmd"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_steering_rate_cmd = res_dict["steering_rate_cmd"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_driving_action = res_dict["driving_action"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_station_error = res_dict["station_error"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_speed_error = res_dict["speed_error"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_lateral_error = res_dict["lateral_error"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_heading_error = res_dict["heading_error"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_control_iteration_cost_ms = res_dict[
        "control_iteration_cost_ms"][vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_localization_jump_count = res_dict["localization_jump_count"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_count_bad_kappa = res_dict["count_bad_kappa"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_count_bad_dkappa = res_dict["count_bad_dkappa"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_fallback_reaction_index = res_dict["fallback_reaction_index"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_chassis_check_code = res_dict["chassis_check_code"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_localization_check_code = res_dict["localization_check_code"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    vehiclestate_trajectory_check_code = res_dict["trajectory_check_code"][
        vehiclestate_valid_start_index:vehiclestate_valid_end_index]
    # data process2
    vehiclestate_plot_time = vehiclestate_timestamp - vehiclestate_timestamp[0]
    vehiclestate_heading_deg = vehiclestate_heading*180/np.pi
    vehiclestate_heading_error_deg = vehiclestate_heading_error*180/np.pi
    vehiclestate_interpoint_distance = [0]
    vehiclestate_heading_diff = [0]
    vehiclestate_heading_diff_deg = [0]
    speed_error_threshold = 0.1*max(vehiclestate_linear_velocity)
    speed_error_threshold = max(speed_error_threshold, 0.2)

    if type(vehiclestate_speed_error[0]) == type('string'):
        if ' nan' in vehiclestate_speed_error or 'nan' in vehiclestate_speed_error or ' nan ' in vehiclestate_speed_error or '':
            print("WARN:speed error has nan!--return--")
            return
        print("speed error has string type! --string to float--")
        vehiclestate_speed_error = list(
            map(StrintToFloat, vehiclestate_speed_error))

    lateral_error_threshold = 0.2
    # print("lateral_error_threshold:", lateral_error_threshold)

    # find the localization jump index and text in xy figure
    localization_jump_index = []
    lat_text_index = []
    lon_text_index = []
    soft_brake_index = []
    has_driving_process = False
    for i in range(0, vehiclestate_valid_length):
        if i > 0:
            dx = vehiclestate_x[i]-vehiclestate_x[i-1]
            dy = vehiclestate_y[i]-vehiclestate_y[i-1]
            dheading = vehiclestate_heading[i]-vehiclestate_heading[i-1]
            vehiclestate_interpoint_distance.append(math.sqrt(dx*dx+dy*dy))
            vehiclestate_heading_diff.append(dheading)
            vehiclestate_heading_diff_deg.append(dheading*180/np.pi)
        if vehiclestate_driving_mode[i] == 1:
            has_driving_process = True
            if i > 0 and vehiclestate_localization_jump_count[i]-vehiclestate_localization_jump_count[i-1] > 0:
                localization_jump_index.append(i)
            if vehiclestate_fallback_reaction_index[i] == 2:
                soft_brake_index.append(i)
            if abs(vehiclestate_lateral_error[i]) > lateral_error_threshold:
                lat_text_index.append(i)
            if abs(vehiclestate_speed_error[i]) > speed_error_threshold or abs(vehiclestate_station_error[i]) > 0.5:
                lon_text_index.append(i)
    if not has_driving_process:
        print("WARN:no autodriving process!")
        print("return")
        return
    average_distance_list = []
    for i in range(0, vehiclestate_valid_length):
        if i < 9:
            average_distance = np.mean(vehiclestate_interpoint_distance[0:19])
        elif i > vehiclestate_valid_length-11:
            average_distance = np.mean(vehiclestate_interpoint_distance[-19:])
        else:
            average_distance = np.mean(
                vehiclestate_interpoint_distance[i-9:i+10])
        average_distance_list.append(average_distance)

    #  find the start valid data index
    start_valid_data_index = 0
    for i in range(0, vehiclestate_valid_length):
        if int(vehiclestate_driving_mode[i]) == 1 and abs(vehiclestate_speed_error[i]) < speed_error_threshold and abs(vehiclestate_lateral_error[i]) < lateral_error_threshold:
            start_valid_data_index = i
            break

    #  find the end valid data index
    end_valid_data_index = start_valid_data_index + 1
    for i in range(vehiclestate_valid_length-1, 0, -1):
        if int(vehiclestate_driving_mode[i]) == 1:
            end_valid_data_index = max(i - 400, start_valid_data_index + 1)
            break

    raw_driving_station_error = vehiclestate_station_error[
        start_valid_data_index:end_valid_data_index]
    raw_driving_speed_error = vehiclestate_speed_error[start_valid_data_index:end_valid_data_index]
    raw_driving_lateral_error = vehiclestate_lateral_error[
        start_valid_data_index:end_valid_data_index]
    raw_driving_heading_error = vehiclestate_heading_error[
        start_valid_data_index:end_valid_data_index]
    raw_driving_control_iteration_cost_ms = vehiclestate_control_iteration_cost_ms[
        start_valid_data_index:end_valid_data_index]
    raw_driving_chassis_check_code = vehiclestate_chassis_check_code[
        start_valid_data_index:end_valid_data_index]
    raw_driving_localization_check_code = vehiclestate_localization_check_code[
        start_valid_data_index:end_valid_data_index]
    raw_driving_trajectory_check_code = vehiclestate_trajectory_check_code[
        start_valid_data_index:end_valid_data_index]
    raw_valid_data_length = len(raw_driving_station_error)
    # delete untrustable data in the process
    # 1) process changing driving mode
    # 2) wierd data like : stationg error is over 10 m
    driving_station_error = []
    driving_speed_error = []
    driving_lateral_error = []
    driving_heading_error = []
    driving_control_iteration_cost_ms = []
    driving_chassis_check_code = []
    driving_loacalization_check_code = []
    driving_trajectory_check_code = []
    flag_back_to_auto = 0
    for i in range(0, raw_valid_data_length):
        if vehiclestate_driving_mode[start_valid_data_index+i] == 0:
            flag_back_to_auto = 0
        elif flag_back_to_auto == 0 and abs(raw_driving_speed_error[i]) < speed_error_threshold and abs(raw_driving_lateral_error[i]) < lateral_error_threshold:
            flag_back_to_auto = 1

        vehiclestate_driving_mode_from_now_on = vehiclestate_driving_mode[
            start_valid_data_index+i+1:]

        next_notdriving_vector = np.where(
            vehiclestate_driving_mode_from_now_on == 0)

        if next_notdriving_vector[0].size == 0:
            next_notdriving_index = 1000
        else:
            next_notdriving_index = next_notdriving_vector[0][0]
        if flag_back_to_auto == 1:
            if next_notdriving_index > 100:
                if abs(raw_driving_lateral_error[i]) < 1:
                    driving_lateral_error.append(raw_driving_lateral_error[i])

                if abs(raw_driving_heading_error[i]) < 0.5:
                    driving_heading_error.append(raw_driving_heading_error[i])

                if abs(raw_driving_station_error[i]) < 10:
                    driving_station_error.append(raw_driving_station_error[i])

                driving_speed_error.append(raw_driving_speed_error[i])
                driving_control_iteration_cost_ms.append(
                    raw_driving_control_iteration_cost_ms[i])
                driving_chassis_check_code.append(
                    raw_driving_chassis_check_code[i])
                driving_loacalization_check_code.append(
                    raw_driving_localization_check_code[i])
                driving_trajectory_check_code.append(
                    raw_driving_trajectory_check_code[i])

    # data screen finished
    if len(driving_station_error) < 50:
        print("not enough control error data--return--")
        return
    abs_driving_station_error = list(map(abs, driving_station_error))
    abs_driving_speed_error = list(map(abs, driving_speed_error))
    abs_driving_lateral_error = list(map(abs, driving_lateral_error))
    abs_driving_heading_error = list(map(abs, driving_heading_error))

    max_station_error = max(abs_driving_station_error)
    max_speed_error = max(abs_driving_speed_error)
    max_later_error = max(abs_driving_lateral_error)
    max_heading_error = max(abs_driving_heading_error) * 180/np.pi
    max_control_iteration_cost_ms = max(driving_control_iteration_cost_ms)

    min_station_error = min(abs_driving_station_error)
    min_speed_error = min(abs_driving_speed_error)
    min_later_error = min(abs_driving_lateral_error)
    min_heading_error = min(abs_driving_heading_error) * 180/np.pi
    min_control_iteration_cost_ms = min(driving_control_iteration_cost_ms)

    mean_station_error = np.mean(abs_driving_station_error)
    mean_speed_error = np.mean(abs_driving_speed_error)
    mean_later_error = np.mean(abs_driving_lateral_error)
    mean_heading_error = np.mean(abs_driving_heading_error) * 180/np.pi
    mean_control_iteration_cost_ms = np.mean(driving_control_iteration_cost_ms)

    var_station_error = np.var(abs_driving_station_error)
    var_speed_error = np.var(abs_driving_speed_error)
    var_later_error = np.var(abs_driving_lateral_error)
    var_heading_error = np.var(abs_driving_heading_error) * 180/np.pi
    var_control_iteration_cost_ms = np.var(driving_control_iteration_cost_ms)

    station_error_tp99 = TPvalue(abs_driving_station_error, 0.99)
    station_error_tp999 = TPvalue(abs_driving_station_error, 0.999)

    speed_error_tp99 = TPvalue(abs_driving_speed_error, 0.99)
    speed_error_tp999 = TPvalue(abs_driving_speed_error, 0.999)

    lateral_error_tp99 = TPvalue(abs_driving_lateral_error, 0.99)
    lateral_error_tp999 = TPvalue(abs_driving_lateral_error, 0.999)

    heading_error_tp99 = TPvalue(abs_driving_heading_error, 0.99)*180/np.pi
    heading_error_tp999 = TPvalue(abs_driving_heading_error, 0.999)*180/np.pi

    control_iteration_cost_ms_tp99 = TPvalue(
        driving_control_iteration_cost_ms, 0.99)
    control_iteration_cost_ms_tp999 = TPvalue(
        driving_control_iteration_cost_ms, 0.999)

    driving_index_list = []
    notdriving_index_list = []

    # separate date from non-driving mode
    for i in range(0, vehiclestate_valid_length):
        # car is on driving mode and not parking state
        if vehiclestate_driving_mode[i] == 1:
            driving_index_list.append(i)
        else:
            notdriving_index_list.append(i)

    tb1 = ptb.PrettyTable()
    tb1.field_names = ["evaluation", "max",
                       "TP999", "TP99", "min", "mean", "variance"]
    tb1.add_row(["station error", max_station_error,
                station_error_tp999, station_error_tp99, min_station_error, round(mean_station_error, 4), round(var_station_error, 4)])
    tb1.add_row(["speed error", max_speed_error,
                speed_error_tp999, speed_error_tp99, min_speed_error, round(mean_speed_error, 4), round(var_speed_error, 4)])
    tb1.add_row(["lateral error", max_later_error,
                lateral_error_tp999, lateral_error_tp99, min_later_error, round(mean_later_error, 4), round(var_later_error, 4)])
    tb1.add_row(["heading error", round(max_heading_error, 4),
                round(heading_error_tp999, 4), round(heading_error_tp99, 4), round(min_heading_error, 4), round(mean_heading_error, 4), round(var_heading_error, 4)])
    tb1.add_row(["control cost time", int(max_control_iteration_cost_ms),
                int(control_iteration_cost_ms_tp999), int(control_iteration_cost_ms_tp99), int(min_control_iteration_cost_ms), round(mean_control_iteration_cost_ms, 4), round(var_control_iteration_cost_ms, 4)])
    print(tb1)

    diagnosis_code(driving_chassis_check_code,
                   driving_loacalization_check_code, driving_trajectory_check_code)
    count_through_percentage = filter_process(
        vehiclestate_steering_angle, output_dir, current_file)

    notdriving_index_matrix = []
    i = 0
    while i < len(notdriving_index_list):
        row = []
        row.append(notdriving_index_list[i])
        i += 1
        while i < len(notdriving_index_list) and notdriving_index_list[i] - notdriving_index_list[i-1] == 1:
            row.append(notdriving_index_list[i])
            i += 1
        notdriving_index_matrix.append(row)
        i += 1
    driving_index_matrix = []
    i = 0
    while i < len(driving_index_list):
        row = []
        row.append(driving_index_list[i])
        i += 1
        while i < len(driving_index_list) and driving_index_list[i] - driving_index_list[i-1] == 1:
            row.append(driving_index_list[i])
            i += 1
        driving_index_matrix.append(row)
        i += 1
    min_driving_x = float("inf")
    min_driving_y = float("inf")
    max_driving_x = float("-inf")
    max_driving_y = float("-inf")
    for i in range(len(driving_index_matrix)):
        driving_index_start = driving_index_matrix[i][0]
        driving_index_end = driving_index_matrix[i][-1]
        min_driving_x = min(min_driving_x, min(
            vehiclestate_x[driving_index_start:driving_index_end+1]))
        min_driving_y = min(min_driving_y, min(
            vehiclestate_y[driving_index_start:driving_index_end+1]))
        max_driving_x = max(max_driving_x, max(
            vehiclestate_x[driving_index_start:driving_index_end+1]))
        max_driving_y = max(max_driving_y, max(
            vehiclestate_y[driving_index_start:driving_index_end+1]))
    steer_angle_through(vehiclestate_x, vehiclestate_y,
                        vehiclestate_steering_angle, vehiclestate_curvature, output_dir, current_file, min_driving_x, max_driving_x, min_driving_y, max_driving_y)
    if enable_plot == 1:
        # Data plot
        # ax1 vehicle_xy
        fig1 = plt.figure(num=1, figsize=(48, 48))
        gs = gridspec.GridSpec(6, 8)
        ax1 = fig1.add_subplot(gs[0:-3, :-4])

        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                ax1.plot(vehiclestate_x[driving_index_start:driving_index_end+1],
                         vehiclestate_y[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto", linewidth=1, marker='.')
            else:
                ax1.plot(vehiclestate_x[driving_index_start:driving_index_end+1],
                         vehiclestate_y[driving_index_start:driving_index_end+1], color='r', linestyle='-', linewidth=1, marker='.')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                ax1.plot(vehiclestate_x[notdriving_index_start],
                         vehiclestate_y[notdriving_index_start], marker='o', markersize=30, color='r')
                ax1.plot(vehiclestate_x[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_y[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                ax1.plot(vehiclestate_x[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_y[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        for i in localization_jump_index:
            if i == localization_jump_index[0]:
                plt.plot(vehiclestate_x[i], vehiclestate_y[i],
                         'X', color='k', markersize=15, label='localization jump')
            else:
                plt.plot(vehiclestate_x[i], vehiclestate_y[i],
                         'X', color='k', markersize=10)
        for i in soft_brake_index:
            if i == soft_brake_index[0]:
                plt.plot(vehiclestate_x[i], vehiclestate_y[i],
                         'o', color='g', markersize=10, label='soft brake')
            else:
                plt.plot(vehiclestate_x[i], vehiclestate_y[i],
                         'o', color='g', markersize=10)
        for i in lat_text_index:
            if i == lat_text_index[0]:
                plt.plot(vehiclestate_x[i], vehiclestate_y[i],
                         '*', color='purple', markersize=10, label='bad lateral')
            else:
                plt.plot(vehiclestate_x[i], vehiclestate_y[i],
                         '*', color='purple', markersize=10)
        for i in lon_text_index:
            if i == lon_text_index[0]:
                plt.plot(vehiclestate_x[i], vehiclestate_y[i],
                         '*', color='c', markersize=10, label='bad longitudinal')
            else:
                plt.plot(vehiclestate_x[i], vehiclestate_y[i],
                         '*', color='c', markersize=10)
        ax1.set_xlabel("X[m]", fontsize=25)
        ax1.set_ylabel("Y[m]", fontsize=25)
        ax1.set_title("Position", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        ax1.axis('equal')
        plt.xlim(min_driving_x-10, max_driving_x+10)
        plt.ylim(min_driving_y-10, max_driving_y+10)

        # ax2 vehicle_heading
        ax2 = fig1.add_subplot(gs[-3:, :-4])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                ax2.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_heading_deg[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                ax2.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_heading_deg[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in localization_jump_index:
            plt.text(vehiclestate_plot_time[i], vehiclestate_heading_deg[i],
                     'X', color='k', fontsize=30)
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                ax2.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_heading_deg[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                ax2.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_heading_deg[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        ax2.set_xlabel("time[s]", fontsize=25)
        ax2.set_ylabel("phi[deg]", fontsize=25)
        ax2.set_title("Heading", fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        plt.legend(loc='best', fontsize=30)
        plt.ylim(-180, 180)
        ax2.grid()

        # distance
        ax3 = fig1.add_subplot(gs[:-3, -4:])
        ax3.plot(vehiclestate_plot_time,
                 vehiclestate_interpoint_distance, color='b', linestyle='-', label='record')
        ax3.plot(vehiclestate_plot_time,
                 average_distance_list, color='r', linestyle='-', label='mean')
        ax3.set_xlabel("time[s]", fontsize=25)
        ax3.set_ylabel("distance[m]", fontsize=25)
        ax3.set_title("interpoint distance", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.ylim(0, 0.1)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        ax3.grid()

        # dheading
        ax4 = fig1.add_subplot(gs[-3:, -4:])
        ax4.plot(vehiclestate_plot_time,
                 vehiclestate_heading_diff_deg, color='r', linestyle='-')
        ax4.set_xlabel("time[s]", fontsize=25)
        ax4.set_ylabel("dheading[deg]", fontsize=25)
        ax4.set_title("heading diff", fontsize=30)
        plt.ylim(-5, 5)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        ax4.grid()

        plt.rcParams['savefig.dpi'] = 300
        plt.rcParams['figure.dpi'] = 300
        plt.savefig(output_dir + "/" + "vehicle_state_" +
                    current_file[-16:-4] + ".png")
        plt.close()
        # bx lon
        fig2 = plt.figure(num=2, figsize=(64, 64))
        gs = gridspec.GridSpec(8, 8)
        bx1 = fig2.add_subplot(gs[0:2, :])
        bx1.plot(vehiclestate_plot_time, vehiclestate_driving_mode,
                 color='b', label='driving mode')
        bx1.plot(vehiclestate_plot_time, vehiclestate_gear,
                 color='r', label='gear')
        bx1.set_xlabel("time[s]", fontsize=25)
        bx1.set_ylabel("state", fontsize=25)
        bx1.set_title("driving state", fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        plt.legend(loc='best', fontsize=30)
        bx1.grid()

        bx2 = fig2.add_subplot(gs[2:4, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                bx2.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_acceleration_cmd[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                bx2.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_acceleration_cmd[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                bx2.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_acceleration_cmd[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                bx2.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_acceleration_cmd[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        bx2.set_xlabel("time[s]", fontsize=25)
        bx2.set_ylabel("acc cmd[m^2/s]", fontsize=25)
        bx2.set_title("acceleration cmd", fontsize=30)
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.legend(loc='best', fontsize=30)
        bx2.grid()

        bx3 = fig2.add_subplot(gs[4:6, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                bx3.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_linear_velocity[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                bx3.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_linear_velocity[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                bx3.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_linear_velocity[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                bx3.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_linear_velocity[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        bx3.set_xlabel("time[s]", fontsize=25)
        bx3.set_ylabel("v[m/s]", fontsize=25)
        bx3.set_title("linear_velocity", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        bx3.grid()

        bx4 = fig2.add_subplot(gs[6:, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                bx4.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_linear_acceleration[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                bx4.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_linear_acceleration[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                bx4.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_linear_acceleration[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                bx4.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_linear_acceleration[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        bx4.set_xlabel("time[s]", fontsize=25)
        bx4.set_ylabel("acc[m^2/s]", fontsize=25)
        bx4.set_title("linear_acceleration", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        bx4.grid()

        plt.rcParams['savefig.dpi'] = 300
        plt.rcParams['figure.dpi'] = 300
        plt.savefig(output_dir + "/" + "vehicle_lon_" +
                    current_file[-16:-4] + ".png")
        plt.close()

        # cx lat
        fig3 = plt.figure(num=3, figsize=(64, 64))
        gs = gridspec.GridSpec(8, 8)
        cx1 = fig3.add_subplot(gs[0:2, :])
        cx1.plot(vehiclestate_plot_time, vehiclestate_driving_mode,
                 color='b', label='driving mode')
        cx1.plot(vehiclestate_plot_time, vehiclestate_gear,
                 color='r', label='gear')
        cx1.set_xlabel("time[s]", fontsize=25)
        cx1.set_ylabel("state", fontsize=25)
        cx1.set_title("driving state", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        cx1.grid()

        cx2 = fig3.add_subplot(gs[2:4, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                cx2.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_steering_cmd[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                cx2.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_steering_cmd[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                cx2.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_steering_cmd[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                cx2.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_steering_cmd[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        cx2.set_xlabel("time[s]", fontsize=25)
        cx2.set_ylabel("steer cmd[deg]", fontsize=25)
        cx2.set_title("steering cmd", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        cx2.grid()

        cx3 = fig3.add_subplot(gs[4:6, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                cx3.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_steering_angle[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                cx3.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_steering_angle[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                cx3.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_steering_angle[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                cx3.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_steering_angle[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        cx3.set_xlabel("time[s]", fontsize=25)
        cx3.set_ylabel("steer angle[deg]", fontsize=25)
        cx3.set_title("steering angle", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        cx3.grid()

        cx4 = fig3.add_subplot(gs[6:, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                cx4.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_steering_angle_spd[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                cx4.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_steering_angle_spd[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                cx4.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_steering_angle_spd[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                cx4.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_steering_angle_spd[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        cx4.set_xlabel("time[s]", fontsize=25)
        cx4.set_ylabel("steering angle spd[deg/s]", fontsize=25)
        cx4.set_title("steering angle speed", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        cx4.grid()

        plt.rcParams['savefig.dpi'] = 300
        plt.rcParams['figure.dpi'] = 300
        plt.savefig(output_dir + "/" + "vehicle_lat_" +
                    current_file[-16:-4] + ".png")
        plt.close()

        # dx lon_err
        fig4 = plt.figure(num=4, figsize=(64, 64))
        gs = gridspec.GridSpec(8, 8)
        dx1 = fig4.add_subplot(gs[0:2, :])
        dx1.plot(vehiclestate_plot_time, vehiclestate_driving_mode,
                 color='b', label='driving mode')
        dx1.plot(vehiclestate_plot_time, vehiclestate_gear,
                 color='r', label='gear')
        dx1.set_xlabel("time[s]", fontsize=25)
        dx1.set_ylabel("state", fontsize=25)
        dx1.set_title("driving state", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        dx1.grid()

        dx2 = fig4.add_subplot(gs[2:4, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                dx2.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_station_error[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                dx2.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_station_error[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                dx2.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_station_error[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                dx2.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_station_error[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        dx2.set_xlabel("time[s]", fontsize=25)
        dx2.set_ylabel("station error[m]", fontsize=25)
        dx2.set_title("station error", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        dx2.grid()

        dx3 = fig4.add_subplot(gs[4:6, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                dx3.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_speed_error[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                dx3.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_speed_error[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                dx3.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_speed_error[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                dx3.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_speed_error[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        dx3.set_xlabel("time[s]", fontsize=25)
        dx3.set_ylabel("speed error[m/s]", fontsize=25)
        dx3.set_title("speed error", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        dx3.grid()

        dx4 = fig4.add_subplot(gs[6:, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                dx4.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_acceleration_cmd[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                dx4.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_acceleration_cmd[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                dx4.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_acceleration_cmd[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                dx4.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_acceleration_cmd[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        dx4.set_xlabel("time[s]", fontsize=25)
        dx4.set_ylabel("acc cmd[m^2/s]", fontsize=25)
        dx4.set_title("acceleration cmd", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        dx4.grid()

        plt.rcParams['savefig.dpi'] = 300
        plt.rcParams['figure.dpi'] = 300
        plt.savefig(output_dir + "/" + "vehicle_lon_err_" +
                    current_file[-16:-4] + ".png")
        plt.close()

        # ex lat_err
        fig5 = plt.figure(num=5, figsize=(64, 64))
        gs = gridspec.GridSpec(8, 8)
        ex1 = fig5.add_subplot(gs[0:2, :])
        ex1.plot(vehiclestate_plot_time, vehiclestate_driving_mode,
                 color='b', label='driving mode')
        ex1.plot(vehiclestate_plot_time, vehiclestate_gear,
                 color='r', label='gear')
        ex1.set_xlabel("time[s]", fontsize=25)
        ex1.set_ylabel("state", fontsize=25)
        ex1.set_title("driving state", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        ex1.grid()

        ex2 = fig5.add_subplot(gs[2:4, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                ex2.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_lateral_error[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                ex2.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_lateral_error[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                ex2.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_lateral_error[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                ex2.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_lateral_error[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        ex2.set_xlabel("time[s]", fontsize=25)
        ex2.set_ylabel("lateral error[m]", fontsize=25)
        ex2.set_title("lateral error", fontsize=30)
        plt.ylim(-0.2, 0.2)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        ex2.grid()

        ex3 = fig5.add_subplot(gs[4:6, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                ex3.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_heading_error_deg[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                ex3.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_heading_error_deg[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                ex3.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_heading_error_deg[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                ex3.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_heading_error_deg[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        ex3.set_xlabel("time[s]", fontsize=25)
        ex3.set_ylabel("heading error[deg]", fontsize=25)
        dx3.set_title("heading error", fontsize=30)
        plt.ylim(-25, 25)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        ex3.grid()

        ex4 = fig5.add_subplot(gs[6:, :])
        for i in range(len(driving_index_matrix)):
            driving_index_start = driving_index_matrix[i][0]
            driving_index_end = driving_index_matrix[i][-1]
            if i == 0:
                ex4.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_steering_cmd[driving_index_start:driving_index_end+1], color='r', linestyle='-', label="auto")
            else:
                ex4.plot(vehiclestate_plot_time[driving_index_start:driving_index_end+1],
                         vehiclestate_steering_cmd[driving_index_start:driving_index_end+1], color='r', linestyle='-')
        for i in range(len(notdriving_index_matrix)):
            notdriving_index_start = notdriving_index_matrix[i][0]
            notdriving_index_end = notdriving_index_matrix[i][-1]
            if i == 0:
                ex4.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_steering_cmd[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-', label="manual")
            else:
                ex4.plot(vehiclestate_plot_time[notdriving_index_start:notdriving_index_end+1],
                         vehiclestate_steering_cmd[notdriving_index_start:notdriving_index_end+1], color='b', linestyle='-')
        ex4.set_xlabel("time[s]", fontsize=25)
        ex4.set_ylabel("steer cmd[deg]", fontsize=25)
        ex4.set_title("steering cmd", fontsize=30)
        plt.legend(loc='best', fontsize=30)
        plt.xlim(0, vehiclestate_plot_time[-1])
        plt.xticks(fontsize=15)
        plt.yticks(fontsize=15)
        ex4.grid()

        plt.rcParams['savefig.dpi'] = 300
        plt.rcParams['figure.dpi'] = 300
        plt.savefig(output_dir + "/" + "vehicle_lat_err_" +
                    current_file[-16:-4] + ".png")
        plt.close()
    del res_dict


def analysis_trajectory(file_name, read_path, save_path):
    print("Analyzing trajectory file :" + file_name)
    current_file = file_name
    input_dir = read_path
    output_dir = save_path
    res_dict = {}

    data = pd.read_csv(input_dir + "/" + current_file)
    headers_list = data.columns.values
    count = 0

    data = data.values
    for header in headers_list:
        res_dict[header.strip()] = data[:, count]
        count += 1
# index, x, y, z, theta, kappa, dkappa, v, a, relative_time, header_time,
    trajectory_index = res_dict["index"][:-2]
    trajectory_x = res_dict["x"][:-2]
    trajectory_y = res_dict["y"][:-2]
    trajectory_z = res_dict["z"][:-2]
    trajectory_theta = res_dict["theta"][:-2]
    trajectory_kappa = res_dict["kappa"][:-2]
    trajectory_dkappa = res_dict["dkappa"][:-2]
    trajectory_v = res_dict["v"][:-2]
    trajectory_a = res_dict["a"][:-2]
    trajectory_relative_time = res_dict["relative_time"][:-2]
    trajectory_header_time = res_dict["header_time"][:-2]
    trajectory_total_length = len(trajectory_x)

    # data process
    color_list = ['k', 'r', 'm', 'g', 'c', 'b', 'y']
    color_length = len(color_list)
    trajectory_theta_deg = trajectory_theta * 180/np.pi
    trajectory_index_int = []
    for i in range(0, trajectory_total_length):
        trajectory_index_int.append(int(trajectory_index[i]))
        if trajectory_relative_time[i] > 100:
            trajectory_relative_time[i] = 0
    trajectory_plot_time = trajectory_header_time - trajectory_header_time[0]
    trajectory_plot_time = trajectory_plot_time + trajectory_relative_time

    if input("Do you want to plot trajectoy pictures? y/n: ") == 'n':
        return
    print("--ploting--")
    # Data plot
    fig1 = plt.figure(num=1, figsize=(64, 64))
    gs = gridspec.GridSpec(6, 8)
    ax1 = fig1.add_subplot(gs[:2, :6])
    ax2 = fig1.add_subplot(gs[2:4, :6])
    ax3 = fig1.add_subplot(gs[4:, :6])
    ax4 = fig1.add_subplot(gs[2:4, 6:])
    current_trajectory_start_index = 0
    current_trajectory_end_index = current_trajectory_start_index

    for index in range(1, max(trajectory_index_int)+1):
        while trajectory_index_int[current_trajectory_start_index] < index:
            current_trajectory_start_index = current_trajectory_start_index + 1
        current_trajectory_end_index = current_trajectory_start_index

        while current_trajectory_end_index < trajectory_total_length-1 and trajectory_index_int[current_trajectory_end_index + 1] < index+1:
            current_trajectory_end_index = current_trajectory_end_index + 1
        number_plot_point_each_instance = 100
        plot_start_index = current_trajectory_start_index
        plot_end_index = min(max(current_trajectory_end_index-100+number_plot_point_each_instance,
                             current_trajectory_start_index+number_plot_point_each_instance-1), trajectory_total_length-1)
        ax1.plot(trajectory_plot_time[plot_start_index:plot_end_index],
                 trajectory_theta_deg[plot_start_index:plot_end_index], color=color_list[index % color_length])
        ax2.plot(trajectory_plot_time[plot_start_index:plot_end_index],
                 trajectory_v[plot_start_index:plot_end_index], color=color_list[index % color_length])
        ax3.plot(trajectory_plot_time[plot_start_index:plot_end_index],
                 trajectory_kappa[plot_start_index:plot_end_index], color=color_list[index % color_length])
        ax4.plot(trajectory_x[plot_start_index:plot_end_index],
                 trajectory_y[plot_start_index:plot_end_index], color=color_list[index % color_length])
        current_trajectory_start_index = current_trajectory_end_index + 1
    ax1.set_xlabel("time[s]", fontsize=25)
    ax1.set_ylabel("theta[deg]", fontsize=25)
    ax1.set_title("trajectory heading", fontsize=30)
    ax1.grid()
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    ax2.set_xlabel("time[s]", fontsize=25)
    ax2.set_ylabel("v[m/s]", fontsize=25)
    ax2.set_title("trajectory velocity", fontsize=30)
    ax2.grid()
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    ax3.set_xlabel("time[s]", fontsize=25)
    ax3.set_ylabel("kappa", fontsize=25)
    ax3.set_title("trajectory curvature", fontsize=30)
    ax3.grid()
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    ax4.set_xlabel("x[m]", fontsize=25)
    ax4.set_ylabel("y[m]", fontsize=25)
    ax4.set_title("trajectory path", fontsize=30)
    ax4.grid()
    ax4.axis('equal')
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    plt.rcParams['savefig.dpi'] = 300
    plt.rcParams['figure.dpi'] = 300
    plt.savefig(output_dir + "/" + "traj_log_" + current_file[-16:-4] + ".png")
    plt.close()
    print("plot succeed")
    del res_dict


def TPvalue(datalist, ratio):
    # ratio is (0,1)
    datalist.sort()
    data_length = len(datalist)
    return_index = math.floor(data_length * ratio)-1
    tpvalue = datalist[return_index]
    return tpvalue


def diagnosis_code(chassis_check_code, localization_check_code, trajectory_check_code):
    # diagnosis code bit
    diag_result_dist = {}
    chassis_nullptr_bit = 1
    chassis_header_bit = 2
    chassis_timestamp_bit = 3
    chassis_timestamp_level_bit = 4
    chassis_gear_bit = 5
    chassis_gear_level_bit = 6
    chassis_speed_bit = 7
    chassis_speed_level_bit = 8
    chassis_steer_bit = 9
    chassis_steer_level_bit = 10
    chassis_driving_mode_bit = 11
    chassis_driving_mode_level_bit = 12
    chassis_summary_bit = 29
    chassis_summary_level_bit = 30
    chassis_unknown_bit = 32

    localization_nullptr_bit = 1
    localization_header_bit = 2
    localization_timestamp_bit = 3
    localization_timestamp_level_bit = 4
    localization_distance_bit = 5
    localization_distance_level_bit = 6
    localization_jump_bit = 7
    localization_jump_level_bit = 8
    localization_status_bit = 9
    localization_status_level_bit = 10
    localization_pose_bit = 11
    localization_pose_level_bit = 12
    localization_summary_bit = 29
    localization_summary_level_bit = 30
    localization_unknown_bit = 32

    trajectory_nullptr_bit = 1
    trajectory_header_bit = 2
    trajectory_timestamp_bit = 3
    trajectory_timestamp_level_bit = 4
    trajectory_gear_bit = 5
    trajectory_gear_level_bit = 6
    trajectory_point_size_bit = 7
    trajectory_point_size_level_bit = 8
    trajectory_relativetime_bit = 9
    trajectory_relativetime_level_bit = 10
    trajectory_kappa_bit = 11
    trajectory_kappa_level_bit = 12
    trajectory_dkappa_bit = 13
    trajectory_dkappa_level_bit = 14
    trajectory_estop_bit = 15
    trajectory_estop_level_bit = 16
    trajectory_summary_bit = 29
    trajectory_summary_level_bit = 30
    trajectory_unknown_bit = 32
    diag_result_dist["chassis_nullptr"] = [chassis_nullptr_bit, 0, 0]
    diag_result_dist["chassis_header"] = [chassis_header_bit, 0, 0]
    diag_result_dist["chassis_timestamp"] = [chassis_timestamp_bit, 0, 0]
    diag_result_dist["chassis_gear"] = [chassis_gear_bit, 0, 0]
    diag_result_dist["chassis_speed"] = [chassis_speed_bit, 0, 0]
    diag_result_dist["chassis_steer"] = [chassis_steer_bit, 0, 0]
    diag_result_dist["chassis_driving_mode"] = [chassis_driving_mode_bit, 0, 0]
    diag_result_dist["chassis_summary"] = [chassis_summary_bit, 0, 0]
    diag_result_dist["chassis_unknown"] = [chassis_unknown_bit, 0, 0]
    diag_result_dist["localization_nullptr"] = [localization_nullptr_bit, 0, 0]
    diag_result_dist["localization_header"] = [localization_header_bit, 0, 0]
    diag_result_dist["localization_timestamp"] = [
        localization_timestamp_bit, 0, 0]
    diag_result_dist["localization_distance"] = [
        localization_distance_bit, 0, 0]
    diag_result_dist["localization_jump"] = [localization_jump_bit, 0, 0]
    diag_result_dist["localization_status"] = [localization_status_bit, 0, 0]
    diag_result_dist["localization_pose"] = [localization_pose_bit, 0, 0]
    diag_result_dist["localization_summary"] = [localization_summary_bit, 0, 0]
    diag_result_dist["localization_unknown"] = [localization_unknown_bit, 0, 0]
    diag_result_dist["trajectory_nullptr"] = [trajectory_nullptr_bit, 0, 0]
    diag_result_dist["trajectory_header"] = [trajectory_header_bit, 0, 0]
    diag_result_dist["trajectory_timestamp"] = [trajectory_timestamp_bit, 0, 0]
    diag_result_dist["trajectory_gear"] = [trajectory_gear_bit, 0, 0]
    diag_result_dist["trajectory_point_size"] = [
        trajectory_point_size_bit, 0, 0]
    diag_result_dist["trajectory_relativetime"] = [
        trajectory_relativetime_bit, 0, 0]
    diag_result_dist["trajectory_kappa"] = [trajectory_kappa_bit, 0, 0]
    diag_result_dist["trajectory_dkappa"] = [trajectory_dkappa_bit, 0, 0]
    diag_result_dist["trajectory_estop"] = [trajectory_estop_bit, 0, 0]
    diag_result_dist["trajectory_summary"] = [trajectory_summary_bit, 0, 0]
    diag_result_dist["trajectory_unknown"] = [trajectory_unknown_bit, 0, 0]

    # dec code to bin code
    data_length = len(chassis_check_code)
    chassis_diagnosis_count = np.zeros([32, 2])
    localization_diagnosis_count = np.zeros([32, 2])
    trajectory_diagnosis_count = np.zeros([32, 2])
    diag_total_count = 0
    print(chassis_check_code[0])
    print(localization_check_code[0])
    print(trajectory_check_code[0])
    for i in range(data_length):
        chassis_check_code_vector = dec2bin_vector(int(chassis_check_code[i]))
        localization_check_code_vector = dec2bin_vector(int(
            localization_check_code[i]))
        trajectory_check_code_vector = dec2bin_vector(
            int(trajectory_check_code[i]))
        if chassis_check_code_vector[chassis_summary_bit-1] == 1 or localization_check_code_vector[localization_summary_bit-1] == 1 or trajectory_check_code_vector[trajectory_summary_bit-1] == 1:
            diag_total_count += 1
        chassis_diagnosis_count[:, 0] = chassis_diagnosis_count[:,
                                                                0] + chassis_check_code_vector[:, 0]
        localization_diagnosis_count[:, 0] = localization_diagnosis_count[:,
                                                                          0] + localization_check_code_vector[:, 0]
        trajectory_diagnosis_count[:, 0] = trajectory_diagnosis_count[:,
                                                                      0] + trajectory_check_code_vector[:, 0]
    chassis_diagnosis_count[:,
                            1] = chassis_diagnosis_count[:, 0] / data_length * 100
    localization_diagnosis_count[:,
                                 1] = localization_diagnosis_count[:, 0] / data_length * 100
    trajectory_diagnosis_count[:,
                               1] = trajectory_diagnosis_count[:, 0] / data_length * 100
    for key in diag_result_dist.keys():
        if key[0:7] == "chassis":
            diag_result_dist[key][1] = chassis_diagnosis_count[diag_result_dist[key][0]-1, 0]
            diag_result_dist[key][2] = chassis_diagnosis_count[diag_result_dist[key][0]-1, 1]
        if key[0:12] == "localization":
            diag_result_dist[key][1] = localization_diagnosis_count[diag_result_dist[key][0]-1, 0]
            diag_result_dist[key][2] = localization_diagnosis_count[diag_result_dist[key][0]-1, 1]
        if key[0:10] == "trajectory":
            diag_result_dist[key][1] = trajectory_diagnosis_count[diag_result_dist[key][0]-1, 0]
            diag_result_dist[key][2] = trajectory_diagnosis_count[diag_result_dist[key][0]-1, 1]
    tb4 = ptb.PrettyTable()
    tb4.field_names = ["Diag result name", "Count", "Percentage(%)"]
    diag_total_percentage = diag_total_count/data_length * 100
    for key in diag_result_dist.keys():
        if not key[-7:] == "summary" and diag_result_dist[key][1] > 0:
            tb4.add_row([key, int(diag_result_dist[key]
                        [1]), round(diag_result_dist[key][2], 4)])
    tb4.add_row(["total", diag_total_count, round(diag_total_percentage, 4)])
    print(tb4)


def dec2bin_vector(code):
    binary_vector_32 = np.zeros([32, 1])
    if code < -pow(2, 31) or code > pow(2, 31)-1:
        return binary_vector_32
    if code < 0:
        binary_vector_32[-1] = 1
        code = abs(code)
    for i in range(len(binary_vector_32)-1):
        binary_vector_32[i] = np.mod(code, 2)
        code = code >> 1
    return binary_vector_32


def filter_process(data, output_dir, current_file):
    filter_order = 7
    frq_sample = 50
    cutoff_frq = 0.2
    wn = cutoff_frq*2/frq_sample
    b, a = ss.butter(N=filter_order, Wn=wn,
                     btype='low')
    # data_after_low_pass = list(ss.filtfilt(b, a, data))
    data_after_low_pass = list(ss.filtfilt(b, a, data))
    frames_shift = math.floor(filter_order/2)
    data_length = len(data_after_low_pass)
    data_after_shift = data_after_low_pass[frames_shift:data_length-1]
    zero_list = [0]*frames_shift
    data_after_shift.extend(zero_list)
    count_through_percentage, bflag_through = CalThroughPercentage(
        data, data_after_shift, frames_shift)

    plt.figure(num=18, figsize=(48, 48))
    plt.plot(data, 'b-', label='raw data')
    plt.plot(data_after_low_pass, 'r-', label='data_after_low_pass')
    plt.plot(data_after_shift, 'g--', label='data_after_shift')
    plt.ylim([-150, 150])
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    plt.legend(loc='best', fontsize=30)
    plt.savefig(output_dir + "/" + "filtered_steer_angle_" +
                current_file[-16:-4] + ".png")
    plt.close()

    return count_through_percentage


def steer_angle_through(x, y, steer_angle, curvature, output_dir, current_file, min_x, max_x, min_y, max_y):
    filter_order = 7
    frq_sample = 50
    cutoff_frq = 0.3
    wn = cutoff_frq*2/frq_sample
    b, a = ss.butter(N=filter_order, Wn=wn,
                     btype='low')
    data = steer_angle
    # data_after_low_pass = list(ss.filtfilt(b, a, data))
    data_after_low_pass = list(ss.filtfilt(b, a, data))
    frames_shift = math.floor(filter_order/2)
    data_length = len(data_after_low_pass)
    data_after_shift = data_after_low_pass[frames_shift:data_length-1]
    zero_list = [0]*frames_shift
    data_after_shift.extend(zero_list)

    index = 0
    for i in range(len(x)):
        if x[i] > min_x and x[i] < max_x and y[i] < max_y and y[i] > min_y:
            index = i
            break
    x = x[index:]
    y = y[index:]
    # Scenario Analysis: turn
    curvature_limit = 0.002
    bflag_turn = np.zeros([data_length, 1])
    for i in range(0, data_length):
        if abs(curvature[i]) >= curvature_limit:
            bflag_turn[i, 0] = 1
    bflag_turn_count = 0
    TurnMarkerList = []
    for i in range(0, len(bflag_turn)):
        if bflag_turn[i, 0] == 1:
            TurnMarkerList.append(i)
            bflag_turn_count += 1
    # Scenario Analysis: drive straight
    straight_deg_limit = 3
    bflag_straight = np.zeros([data_length, 1])
    bflag_straight_count = 0
    StraightMarkerList = []
    data_after_shift_straight = []
    x_straight = []
    y_straight = []
    for i in range(0, data_length-frames_shift):
        if abs(data_after_shift[i]) < straight_deg_limit:
            bflag_straight[i, 0] = 1
            StraightMarkerList.append(i)
            data_after_shift_straight.append(data_after_shift[i])
            if i >= index:
                x_straight.append(x[i-index])
                y_straight.append(y[i-index])
            bflag_straight_count += 1
    # count through
    limit = [0, 1, 3, 5]
    through_percentage = [0]*len(limit)
    bflag_through = np.zeros([data_length, len(limit)])
    bflag_through_T = bflag_through.T
    for i in range(0, len(limit)):
        if limit[i] == 0:
            through_percentage[i], bflag_through_T[i, :] = CalThroughPercentage(
                steer_angle, data_after_shift, frames_shift)
            continue
        data_after_shift_addlimit = []
        data_after_shift_minnuslimit = []
        for j in range(len(data_after_shift)):
            data_after_shift_addlimit.append(data_after_shift[j]+limit[i])
            data_after_shift_minnuslimit.append(data_after_shift[j]-limit[i])
        through_percentage_positive, bflag_through_positive = CalThroughPercentage(
            steer_angle, data_after_shift_addlimit, frames_shift)
        through_percentage_negtive, bflag_through_negtive = CalThroughPercentage(
            steer_angle, data_after_shift_minnuslimit, frames_shift)
        through_percentage[i] = through_percentage_positive + \
            through_percentage_negtive
        bflag_through_T[i, :] = bflag_through_positive + bflag_through_negtive
    bflag_through = bflag_through_T.T
    steer_angle_limit = limit
    total_through_percentage = through_percentage
    # straight_through_percentage
    count_straight_through = np.dot(bflag_straight.T, bflag_through)
    straight_through_percentage = count_straight_through.T/bflag_straight_count*100
    # turn_through_percentage
    count_turn_through = np.dot(bflag_turn.T, bflag_through)
    turn_through_percentage = count_turn_through.T/bflag_turn_count*100
    tb5 = ptb.PrettyTable()
    tb5.field_names = ["steer_angle_limit",
                       "total_through(%)", "straight_through(%)", "turn_through(%)"]
    for i in range(len(limit)):
        tb5.add_row([steer_angle_limit[i], round(total_through_percentage[i], 4),
                    round(straight_through_percentage[i, 0], 4), round(turn_through_percentage[i, 0], 4)])
    print(tb5)

    fig1 = plt.figure(num=19, figsize=(64, 64))
    gs = gridspec.GridSpec(8, 8)
    ax1 = fig1.add_subplot(gs[:4, :])
    ax2 = fig1.add_subplot(gs[4:, :])

    ax1.plot(data, label='raw data', color='b')
    ax1.plot(data_after_shift, label='filtered data',
             color='k', linestyle='--')
    ax2.plot(x, y, label='raw data', color='b')
    continuous_start_index = 0
    continuous_end_index = continuous_start_index + 1
    i = 0
    while i < len(StraightMarkerList)-1:
        continuous_start_index = StraightMarkerList[i]
        while i < len(StraightMarkerList)-1 and StraightMarkerList[i+1] == StraightMarkerList[i] + 1:
            i += 1
        continuous_end_index = StraightMarkerList[i]
        i += 1
        plot_index = range(continuous_start_index, continuous_end_index)
        ax1.plot(
            plot_index, data_after_shift[continuous_start_index:continuous_end_index], 'r-', linewidth=5)
        if i > index:
            ax2.plot(
                x[continuous_start_index-index:continuous_end_index-index], y[continuous_start_index-index:continuous_end_index-index], 'r-', linewidth=5)
        continuous_start_index = max(
            continuous_end_index+1, continuous_start_index+1)
    i = 0
    while i < len(TurnMarkerList)-1:
        continuous_start_index = TurnMarkerList[i]
        while i < len(TurnMarkerList)-1 and TurnMarkerList[i+1] == TurnMarkerList[i] + 1:
            i += 1
        continuous_end_index = TurnMarkerList[i]
        i += 1
        plot_index = range(continuous_start_index, continuous_end_index)
        ax1.plot(
            plot_index, data_after_shift[continuous_start_index:continuous_end_index], 'y-', linewidth=5)
        if i > index:
            ax2.plot(
                x[continuous_start_index-index:continuous_end_index-index], y[continuous_start_index-index:continuous_end_index-index], 'y-', linewidth=5)
        continuous_start_index = max(
            continuous_end_index+1, continuous_start_index+1)
    ax1.legend(loc='best', fontsize=30)
    ax1.set_title("steer angle through")

    ax2.legend(loc='best', fontsize=30)
    ax2.set_title("position")
    ax2.axis('equal')
    # plt.xlim(min_x, max_x)
    # plt.ylim(min_y, max_y)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)
    plt.savefig(output_dir + "/" + "steer_through_" +
                current_file[-16:-4] + ".png")
    plt.close()


def CalThroughPercentage(raw_data, data_after_shift, frames_shift):
    i = 0
    cur_err = raw_data[i] - data_after_shift[i]
    if cur_err == 0:
        is_last_point_equel = 1
    else:
        is_last_point_equel = 0
    total_size = len(raw_data)
    bflag_through = np.zeros([total_size, 1])
    count_through = 0
    for i in range(1, total_size-frames_shift):
        cur_err = raw_data[i] - data_after_shift[i]
        last_err = raw_data[i-1] - data_after_shift[i-1]
        if is_last_point_equel == 1 and not cur_err == 0.0:
            count_through = count_through + 1
            bflag_through[i-1] = 1
            is_last_point_equel = 0
        if cur_err == 0.0:
            is_last_point_equel = 1
        if (cur_err > 0 and last_err < 0) or (cur_err < 0 and last_err > 0):
            count_through = count_through + 1
            bflag_through[i] = 1
    through_percentage = count_through/(total_size-frames_shift)*100
    return through_percentage, bflag_through.T


def StrintToFloat(str):
    return float(str)


def main():
    analysis_v2()


if __name__ == '__main__':
    main()
