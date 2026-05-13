import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
import prettytable as ptb
import os
import sys
import math
import csv


def analysis(data_dir="/data", result_dir="/result"):
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
    file_list = os.listdir(input_dir)
    if file_list == []:
        print("No files found in input directory. ")
        return

    # Read files & process
    for current_file in file_list:

        # Ignore invalid files
        if not current_file[-4:] == ".txt":
            continue

        if current_file[:6] == "Fusion":
            analysis_fusion(current_file, input_dir, output_dir)

    # Read csv files & process
    for current_file in file_list:
        # Ignore invalid files
        if not current_file[-4:] == ".csv":
            continue

        if current_file[:23] == "lateral_tuning_log_data":
            analysis_log_data(current_file, input_dir, output_dir)

        if current_file[:19] == "lateral_tuning_data":
            analysis_data(current_file, input_dir, output_dir)

        if current_file[:4] == "havp":
            analysis_havp(current_file, input_dir, output_dir)


def analysis_log_data(file_name, read_path, save_path):
    print("---------------------------------------------------------------")
    print("Analyzing lateral_tuning_log_data:" + file_name)
    current_file = file_name
    input_dir = read_path
    output_dir = save_path
    res_dict = {}

    # Init some dicts to identify data
    # (x, y, z, qx, qy, qz, qw, heading, velocity, accel)
    data = pd.read_csv(input_dir + "/" + current_file)
    data = data.values
    data = data.T
    x = data[0]
    y = data[1]
    phi = data[7]
    phi_deg = phi * 180/np.pi
    data_length = len(x)
    interpoint_distance = [0]
    delta_phi_deg = [0]
    lateral_error = []
    lat_index = []
    for i in range(1, data_length-1):
        distance = math.sqrt(
            (x[i]-x[i-1])*(x[i]-x[i-1]) + (y[i]-y[i-1])*(y[i]-y[i-1]))
        interpoint_distance.append(distance)
    for i in range(1, data_length-1):
        delta_phi_deg.append(phi_deg[i]-phi_deg[i-1])
    for i in range(0, data_length-1):
        dx = abs(x[i+1] - x[i])
        dy = abs(y[i+1] - y[i])
        current_heading = phi[i]
        if current_heading > -np.pi/2 and current_heading < np.pi/2:
            alpha = abs(current_heading)
        else:
            alpha = np.pi - abs(current_heading)
        lat_error = math.cos(alpha) * dy - math.sin(alpha) * dx
        lateral_error.append(lat_error)
        if abs(lat_error) > 0.2:
            lat_index.append(i)
    lateral_error.append(0)
    distance_count = 0
    average_distance_list = []
    for i in range(0, data_length-1):
        if i < 9:
            average_distance = np.mean(interpoint_distance[0:19])
        elif i > data_length-11:
            average_distance = np.mean(interpoint_distance[-19:])
        else:
            average_distance = np.mean(interpoint_distance[i-9:i+10])
        average_distance_list.append(average_distance)
        if abs(interpoint_distance[i]-average_distance) > 0.05:
            distance_count = distance_count+1

    tb1 = ptb.PrettyTable()
    tb1.field_names = ["   ", "count", "ratio"]
    tb1.add_row(["sum", data_length, 1])
    tb1.add_row(["distance diff over 0.1", distance_count,
                distance_count/data_length])
    tb1.add_row(["lateral error over 0.2", len(
        lat_index), len(lat_index)/data_length])
    print(tb1)

    fig = plt.figure(num=1, figsize=(48, 48))
    gs = gridspec.GridSpec(6, 8)

    ax1 = fig.add_subplot(gs[0:-3, :-4])
    ax1.plot(x, y)
    ax1.set_xlabel("X[m]", fontsize=25)
    ax1.set_ylabel("Y[m]", fontsize=25)
    ax1.set_title("Position", fontsize=30)
    ax1.axis('equal')
    ax1.grid()
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    ax2 = fig.add_subplot(gs[-3:, :-4])
    ax2.plot(interpoint_distance, color='b', label='interpoint_distance')
    ax2.plot(average_distance_list, color='r', label='average_distance')
    ax2.set_xlabel("index", fontsize=25)
    ax2.set_ylabel("distance[m]", fontsize=25)
    ax2.set_title("interpoint distance", fontsize=30)
    ax2.legend()
    ax2.grid()
    plt.ylim(0, 1.0)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    ax3 = fig.add_subplot(gs[0:-3, -4:])
    ax3.plot(phi_deg, color='b')
    ax3.set_xlabel("index", fontsize=25)
    ax3.set_ylabel("phi deg[deg]", fontsize=25)
    ax3.set_title("heading", fontsize=30)
    ax3.grid()
    plt.ylim(-180, 180)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    ax4 = fig.add_subplot(gs[-3:, -4:])
    ax4.plot(delta_phi_deg, color='b')
    ax4.set_xlabel("index", fontsize=25)
    ax4.set_ylabel("delta phi deg[deg]", fontsize=25)
    ax4.grid()
    plt.ylim(-3, 3)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    plt.savefig(output_dir + "/" + current_file[:-4] + ".png")
    plt.close()

    fig2 = plt.figure(num=2, figsize=(48, 48))
    gs = gridspec.GridSpec(6, 8)

    bx1 = fig2.add_subplot(gs[0:-3, :-4])
    bx1.plot(x, y)
    for index in lat_index:
        plt.text(x[index], y[index], "X", fontsize=30, color='r')
    bx1.set_xlabel("X[m]", fontsize=25)
    bx1.set_ylabel("Y[m]", fontsize=25)
    bx1.set_title("position", fontsize=30)
    bx1.axis('equal')
    bx1.grid()
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    bx2 = fig2.add_subplot(gs[-3:, :-4])
    bx2.plot(lateral_error)
    for index in lat_index:
        plt.text(index, lateral_error[index], "X", fontsize=30, color='r')
    bx2.set_xlabel("index", fontsize=25)
    bx2.set_ylabel("lateral error", fontsize=25)
    bx2.set_title("lateral error", fontsize=30)
    bx2.grid()
    plt.ylim(-0.3, 0.3)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    plt.savefig(output_dir + "/" + current_file[:-4] + "_lateral.png")
    plt.close()


def analysis_data(file_name, read_path, save_path):
    print("---------------------------------------------------------------")
    print("Analyzing lateral_tuning_data file :" + file_name)
    current_file = file_name
    input_dir = read_path
    output_dir = save_path
    res_dict = {}

    # Init some dicts to identify data
    #  (x,y,z,heading,kappa,dkappa, s, left_bound, right_bound)
    data = pd.read_csv(input_dir + "/" + current_file)
    data = data.values
    data = data.T
    x = data[0]
    y = data[1]
    phi = data[3]
    curvature = data[4]
    phi_deg = phi * 180/np.pi
    data_length = len(x)
    interpoint_distance = [0]
    delta_phi_deg = [0]
    lateral_error = [0]
    lat_index = []
    for i in range(1, data_length-1):
        distance = math.sqrt(
            (x[i]-x[i-1])*(x[i]-x[i-1]) + (y[i]-y[i-1])*(y[i]-y[i-1]))
        interpoint_distance.append(distance)
    for i in range(1, data_length-1):
        delta_phi_deg.append(phi_deg[i]-phi_deg[i-1])
    for i in range(0, data_length-1):
        dx = abs(x[i+1] - x[i])
        dy = abs(y[i+1] - y[i])
        current_heading = phi[i]
        if current_heading > -np.pi/2 and current_heading < np.pi/2:
            alpha = abs(current_heading)
        else:
            alpha = np.pi - abs(current_heading)
        lat_error = math.cos(alpha) * dy - math.sin(alpha) * dx
        lateral_error.append(lat_error)
        if abs(lat_error) > 0.2:
            lat_index.append(i)
    average_distance_list = []
    distance_count = 0
    for i in range(0, data_length-1):
        if i < 9:
            average_distance = np.mean(interpoint_distance[0:19])
        elif i > data_length-11:
            average_distance = np.mean(interpoint_distance[-19:])
        else:
            average_distance = np.mean(interpoint_distance[i-9:i+10])
        average_distance_list.append(average_distance)
        if abs(interpoint_distance[i]-average_distance) > 0.05:
            distance_count = distance_count+1
    heading_count = 0
    is_last_heading_normal = 1
    for i in range(1, data_length-1):
        if is_last_heading_normal == 1 and abs(delta_phi_deg[i]) > 1 and abs(delta_phi_deg[i]) < 179:
            heading_count = heading_count + 1
            is_last_heading_normal = 0
        elif abs(delta_phi_deg[i]) < 0.3:
            is_last_heading_normal = 1
    tb1 = ptb.PrettyTable()
    tb1.field_names = ["   ", "count", "ratio"]
    tb1.add_row(["sum", data_length, 1])
    tb1.add_row(["delta phi over 1 deg", heading_count,
                heading_count/data_length])
    print(tb1)

    fig = plt.figure(num=1, figsize=(48, 48))
    gs = gridspec.GridSpec(6, 8)

    ax1 = fig.add_subplot(gs[0:-3, :-4])
    ax1.plot(x, y)
    ax1.set_xlabel("X[m]", fontsize=25)
    ax1.set_ylabel("Y[m]", fontsize=25)
    ax1.set_title("Position", fontsize=30)
    ax1.axis('equal')
    ax1.grid()

    ax2 = fig.add_subplot(gs[-3:, :-4])
    ax2.plot(interpoint_distance, color='b', label='interpoint_distance')
    ax2.plot(average_distance_list, color='r', label='average_distance')
    ax2.set_xlabel("index", fontsize=25)
    ax2.set_ylabel("distance[m]", fontsize=25)
    ax2.set_title("interpoint distance", fontsize=30)
    ax2.grid()
    plt.ylim(0, 1.0)

    ax3 = fig.add_subplot(gs[0:-3, -4:])
    ax3.plot(phi_deg, color='b')
    ax3.set_xlabel("index", fontsize=25)
    ax3.set_ylabel("phi deg[deg]", fontsize=25)
    ax3.set_title("heading", fontsize=30)
    ax3.grid()
    plt.ylim(-180, 180)

    ax4 = fig.add_subplot(gs[-3:, -4:])
    ax4.plot(delta_phi_deg, color='b')
    ax4.set_xlabel("index", fontsize=25)
    ax4.set_ylabel("delta phi deg[deg]", fontsize=25)
    ax4.set_title("delta heading", fontsize=30)
    ax4.grid()
    plt.ylim(-3, 3)

    plt.savefig(output_dir + "/" + current_file[:-4] + ".png")
    plt.close()

    fig2 = plt.figure(num=2, figsize=(48, 48))
    gs = gridspec.GridSpec(6, 8)

    bx1 = fig2.add_subplot(gs[0:-3, :-4])
    bx1.plot(x, y)
    for index in lat_index:
        plt.text(x[index], y[index], "X", fontsize=20, color='r')
    bx1.set_xlabel("X[m]", fontsize=25)
    bx1.set_ylabel("Y[m]", fontsize=25)
    bx1.set_title("position", fontsize=30)
    bx1.axis('equal')
    bx1.grid()

    bx2 = fig2.add_subplot(gs[-3:, :-4])
    bx2.plot(lateral_error)
    for index in lat_index:
        plt.text(index, lateral_error[index], "X", fontsize=20, color='r')
    bx2.set_xlabel("index", fontsize=25)
    bx2.set_ylabel("lateral error", fontsize=25)
    bx2.set_title("lateral error", fontsize=30)
    bx2.grid()
    plt.ylim(-0.3, 0.3)

    bx3 = fig2.add_subplot(gs[0:-3, -4:])
    bx3.plot(curvature, color='b')
    bx3.set_xlabel("index", fontsize=25)
    bx3.set_ylabel("curvature", fontsize=25)
    bx3.set_title("curvature", fontsize=30)
    bx3.grid()
    plt.ylim(-0.2, 0.2)

    plt.savefig(output_dir + "/" + current_file[:-4] + "_lateral.png")
    plt.close()


def analysis_fusion(file_name, read_path, save_path):
    print("---------------------------------------------------------------")
    print("Analyzing :" + file_name)
    current_file = file_name
    input_dir = read_path
    output_dir = save_path
    data = []
    f = open(input_dir + "/" + current_file, 'r')
    for line in f:
        data_line = line.strip("\n").split()
        data.append([float(i) for i in data_line])
    f.close()
    data = np.array(data)
    data = data.T
    timestamp_sec = data[0]
    x = data[1]
    y = data[2]
    z = data[3]
    qx = data[4]
    qy = data[5]
    qz = data[6]
    qw = data[7]
    time = timestamp_sec - timestamp_sec[0]
    phi = np.zeros([len(x), 1])
    for i in range(0, len(x)):
        roll = math.atan2((2.0) * (qw[i] * qy[i] - qx[i] * qz[i]),
                          (2.0) * (pow(qw[i], 2) + pow(qz[i], 2)) - (1.0))

        pitch = math.asin((2.0) * (qw[i] * qx[i] + qy[i] * qz[i]))

        yaw = math.atan2((2.0) * (qw[i] * qz[i] - qx[i] * qy[i]),
                         (2.0) * (pow(qw[i], 2) + pow(qy[i], 2)) - (1.0))
        yaw = np.mod(yaw + np.pi, 2.0 * np.pi)
        if yaw < 0:
            phi[i, 0] = yaw + np.pi
        else:
            phi[i, 0] = yaw - np.pi
    # # write csv x,y,heading
    log_file = input_dir + "/" + "lateral_tuning_log_data_Fusion.csv"
    csv_file = open(log_file, 'a+', encoding='utf-8', newline='')
    csv_writer = csv.writer(csv_file)
    for i in range(len(x)):
        # planning sample 10Hz , data source 100Hz
        if i % 10 == 0:
            csv_writer.writerow(
                [x[i], y[i], z[i], qx[i], qy[i], qz[i], qw[i], phi[i, 0], 0.0, 0.0])
    csv_file.close()

    # data process
    phi_deg = phi * 180/np.pi
    data_length = len(x)
    interpoint_distance = [0]
    delta_phi_deg = [0]

    lateral_error = []
    lat_index = []
    for i in range(1, data_length):
        distance = math.sqrt(
            (x[i]-x[i-1])*(x[i]-x[i-1]) + (y[i]-y[i-1])*(y[i]-y[i-1]))
        interpoint_distance.append(distance)
    for i in range(1, data_length):
        delta_phi_deg.append(phi_deg[i, 0]-phi_deg[i-1, 0])
    for i in range(0, data_length-1):
        dx = abs(x[i+1] - x[i])
        dy = abs(y[i+1] - y[i])
        current_heading = phi[i]
        if current_heading > -np.pi/2 and current_heading < np.pi/2:
            alpha = abs(current_heading)
        else:
            alpha = np.pi - abs(current_heading)
        lat_error = math.cos(alpha) * dy - math.sin(alpha) * dx
        lateral_error.append(lat_error)
        if abs(lat_error) > 0.2:
            lat_index.append(i)
    lateral_error.append(0)
    distance_count = 0
    average_distance_list = []
    for i in range(0, data_length-1):
        if i < 9:
            average_distance = np.mean(interpoint_distance[0:19])
        elif i > data_length-11:
            average_distance = np.mean(interpoint_distance[-19:])
        else:
            average_distance = np.mean(interpoint_distance[i-9:i+10])
        average_distance_list.append(average_distance)
        if abs(interpoint_distance[i]-average_distance) > 0.05:
            distance_count = distance_count+1

    tb1 = ptb.PrettyTable()
    tb1.field_names = ["   ", "count", "ratio"]
    tb1.add_row(["sum", data_length, 1])
    tb1.add_row(["distance diff over 0.1", distance_count,
                distance_count/data_length])
    tb1.add_row(["lateral error over 0.2", len(
        lat_index), len(lat_index)/data_length])
    print(tb1)

    fig = plt.figure(num=1, figsize=(48, 48))
    gs = gridspec.GridSpec(6, 8)

    ax1 = fig.add_subplot(gs[0:-3, :-4])
    ax1.plot(x, y)
    ax1.set_xlabel("X[m]", fontsize=25)
    ax1.set_ylabel("Y[m]", fontsize=25)
    ax1.set_title("Position", fontsize=30)
    ax1.axis('equal')
    ax1.grid()
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    ax2 = fig.add_subplot(gs[-3:, :-4])
    ax2.plot(interpoint_distance, color='b', label='interpoint_distance')
    ax2.plot(average_distance_list, color='r', label='average_distance')
    ax2.set_xlabel("index", fontsize=25)
    ax2.set_ylabel("distance[m]", fontsize=25)
    ax2.set_title("interpoint distance", fontsize=30)
    ax2.legend()
    ax2.grid()
    plt.ylim(0, 0.1)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    ax3 = fig.add_subplot(gs[0:-3, -4:])
    ax3.plot(phi_deg, color='b')
    ax3.set_xlabel("index", fontsize=25)
    ax3.set_ylabel("phi deg[deg]", fontsize=25)
    ax3.set_title("heading", fontsize=30)
    ax3.grid()
    plt.ylim(-180, 180)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    ax4 = fig.add_subplot(gs[-3:, -4:])
    ax4.plot(delta_phi_deg, color='b')
    ax4.set_xlabel("index", fontsize=25)
    ax4.set_ylabel("delta phi deg[deg]", fontsize=25)
    ax4.grid()
    plt.ylim(-3, 3)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    plt.savefig(output_dir + "/" + current_file[:-4] + ".png")
    plt.close()

    fig2 = plt.figure(num=2, figsize=(48, 48))
    gs = gridspec.GridSpec(6, 8)

    bx1 = fig2.add_subplot(gs[0:-3, :-4])
    bx1.plot(x, y)
    for index in lat_index:
        plt.text(x[index], y[index], "X", fontsize=30, color='r')
    bx1.set_xlabel("X[m]", fontsize=25)
    bx1.set_ylabel("Y[m]", fontsize=25)
    bx1.set_title("position", fontsize=30)
    bx1.axis('equal')
    bx1.grid()
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    bx2 = fig2.add_subplot(gs[-3:, :-4])
    bx2.plot(lateral_error)
    for index in lat_index:
        plt.text(index, lateral_error[index], "X", fontsize=30, color='r')
    bx2.set_xlabel("index", fontsize=25)
    bx2.set_ylabel("lateral error", fontsize=25)
    bx2.set_title("lateral error", fontsize=30)
    bx2.grid()
    plt.ylim(-0.1, 0.1)
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    bx3 = fig2.add_subplot(gs[:-3, -4:])
    bx3.plot(time)
    bx3.set_xlabel("index", fontsize=25)
    bx3.set_ylabel("time[s]", fontsize=25)
    bx3.set_title("timestamp", fontsize=30)
    bx3.grid()
    plt.xticks(fontsize=15)
    plt.yticks(fontsize=15)

    plt.savefig(output_dir + "/" + current_file[:-4] + "_lateral.png")
    plt.close()


def analysis_havp(file_name, read_path, save_path):
    print("---------------------------------------------------------------")
    print("Analyzing lateral_tuning_data file :" + file_name)
    current_file = file_name
    input_dir = read_path
    output_dir = save_path
    res_dict = {}

    # Init some dicts to identify data
    #  (x,y,z,heading,kappa,dkappa, s, left_bound, right_bound)
    data = pd.read_csv(input_dir + "/" + current_file)
    data = data.values
    data = data.T
    x = data[0]
    y = data[1]
    phi = data[2]
    curvature = data[3]
    phi_deg = phi * 180/np.pi

    
    data_length = len(x)
    delta_phi_deg = [0]
    lateral_error = [0]
    lat_index = []
    #calculate delta phi in degree
    for i in range(0, data_length - 1):
        if i == 0:
            d_phi_deg=phi_deg[0] - 0
        else:
            d_phi_deg = phi_deg[i] - phi_deg[i-1]

        if d_phi_deg > 180:
            d_phi_deg = d_phi_deg - 360
        elif d_phi_deg < -180:
            d_phi_deg = d_phi_deg + 360
        delta_phi_deg.append(d_phi_deg)
    #calculate lateral error
    for i in range(0, data_length - 1):
        dx = abs(x[i+1] - x[i])
        dy = abs(y[i+1] - y[i])
        current_heading = phi[i]
        if current_heading > -np.pi/2 and current_heading < np.pi/2:
            alpha = abs(current_heading)
        else:
            alpha = np.pi - abs(current_heading)
        lat_error = math.cos(alpha) * dy - math.sin(alpha) * dx
        lateral_error.append(lat_error)
        if abs(lat_error) > 0.2:
            lat_index.append(i)

    end = 0
    over_index = []
    over_curvature = []
    over_x = []
    over_y = []
    for i in range(0, data_length - 1):
        if i==0 or i<end:
            continue
        if abs(curvature[i]) >= 0.05:
            for j in range(i, data_length-1):
                if abs(curvature[j]) <= 0.05:
                    end = j
                    break
        if end-i < 15:
            for k in range(i, end):
                over_x.append(x[k])
                over_y.append(y[k])
                over_index.append(k)
                over_curvature.append(curvature[k])

    fig = plt.figure(num=1, figsize=(48, 48))
    gs = gridspec.GridSpec(6, 8)

    ax1 = fig.add_subplot(gs[0:-3, :-4])
    ax1.plot(x, y)
    ax1.set_xlabel("X[m]", fontsize=25)
    ax1.set_ylabel("Y[m]", fontsize=25)
    ax1.set_title("Position", fontsize=30)
    ax1.axis('equal')
    ax1.grid()
    ax1.scatter(over_x, over_y, c = 'r')

    ax2 = fig.add_subplot(gs[-3:, :-4])
    ax2.plot(curvature, color='b', label='curvature')
    ax2.set_xlabel("index", fontsize=25)
    ax2.set_ylabel("curvature", fontsize=25)
    ax2.set_title("curvature", fontsize=30)
    ax2.scatter(over_index,over_curvature,c='r')
    for i in over_index:
        ax2.annotate(f'({curvature[i]})', xy=(i, curvature[i]), fontsize = 7.5)

    ax2.grid()

    ax3 = fig.add_subplot(gs[0:-3, -4:])
    ax3.plot(phi_deg, color='b')
    ax3.set_xlabel("index", fontsize=25)
    ax3.set_ylabel("phi deg[deg]", fontsize=25)
    ax3.set_title("heading", fontsize=30)
    ax3.grid()
    plt.ylim(-180, 180)

    #plot delta phi degree
    ax4 = fig.add_subplot(gs[-3:, -4:])
    ax4.plot(delta_phi_deg, color = 'b')
    ax4.set_xlabel("index", fontsize = 25)
    ax4.set_ylabel("delta phi deg[deg]", fontsize = 25)
    ax4.set_title("delta heading", fontsize = 30)
    ax4.grid()
    plt.ylim(-3, 3)

    plt.savefig(output_dir + "/" + current_file + ".png")
    plt.close()


    fig2 = plt.figure(num = 2, figsize = (48, 48))
    gs = gridspec.GridSpec(6, 8)

    #plot lateral error
    bx1 = fig2.add_subplot(gs[0:-3, :-4])
    bx1.plot(lateral_error)
    for index in lat_index:
        plt.text(index, lateral_error[index], "x", fontsize = 20, color = 'r')
    bx1.set_xlabel("index", fontsize = 25)
    bx1.set_ylabel("lateral error", fontsize = 25)
    bx1.set_title("lateral error", fontsize = 30)
    bx1.grid()
    plt.ylim(-0.025, 0.025)

    plt.savefig(output_dir + "/" + current_file[:-4] + "_lateral.png")
    plt.close()
    


def main():
    analysis()


if __name__ == '__main__':
    main()
