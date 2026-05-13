%% main function 
%% JIDU Auto PnC
% liang.zhu@jiduauto.com
clear;
close all
%%
common_mat
%%  1添加到路径,2此处更改时间戳后缀
% special group analysis
trajectory_filename = 'trajectory_log_2023-08-24_110906.csv';
control_filename = 'vehicle_state_and_command_log_2023-08-24_110904.csv';
% batch data analysis
analysis_dir = "csv_0824";
%%
if batch_data_analysis ==0

control_csv_analysis

%% batch data analysis, offer evaluation only for csv files in certain dir
else 
path_cell = strsplit(pwd,'/');
path_tail = path_cell{end};
% for user: You May Need to Mannually Change the Dir Name Below To Suit!!!
if path_tail == "controlm"
    files_list = dir(pwd + "/control_analysis/" + analysis_dir);
elseif path_tail == "control_analysis"
    files_list = dir(pwd + "/" + analysis_dir);
end
trajectory_csv_list = [];
vehiclestate_csv_list = [];
for i = 1:length(files_list)
    cur_csv_name = files_list(i).name;
    if(contains(cur_csv_name,"vehicle_state"))
        vehiclestate_csv_list = [vehiclestate_csv_list;string(cur_csv_name)];
    end
    if(contains(cur_csv_name,"trajectory"))
        trajectory_csv_list = [trajectory_csv_list;string(cur_csv_name)];
    end
end
for i =1:length(trajectory_csv_list)
    cur_trajectory_csv = char(trajectory_csv_list(i));
    cur_trajectory_csv_datestr = cur_trajectory_csv(end-12:end-11);
    cur_trajectory_csv_timestr = cur_trajectory_csv(end-9:end-4);
    trajectory_filename = cur_trajectory_csv;
    for j = 1:length(vehiclestate_csv_list)
        cur_vehiclestate_csv = char(vehiclestate_csv_list(j));
        cur_vehiclestate_csv_datestr = cur_vehiclestate_csv(end-12:end-11);
        cur_vehiclestate_csv_timestr = cur_vehiclestate_csv(end-9:end-4);
        if string(cur_vehiclestate_csv_datestr) ~= string(cur_trajectory_csv_datestr)
            continue
        end
        time_delay_vector = cur_trajectory_csv_timestr - cur_vehiclestate_csv_timestr;
        time_weight_vector = [36000;3600;600;60;10;1];
        time_delay = time_delay_vector * time_weight_vector;
        if time_delay >=0 && time_delay < 20
            control_filename = cur_vehiclestate_csv;
            break;
        end
    end
    control_csv_analysis
end
end