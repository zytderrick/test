%% input data is vehiclestate data
%%
function f_data_evaluate(vehiclestate)
load("common.mat");
%% 
vehiclestate_time = vehiclestate(:,timestamp_sec_index);
vehiclestate = vehiclestate(find(vehiclestate_time > 1e-1),:);
vehiclestate_valid_length = length(vehiclestate);

vehiclestate_localization_velocity = vehiclestate(:,localization_velocity_index);
vehiclestate_linear_velocity = vehiclestate(:,linear_velocity_index);
vehiclestate_gear = vehiclestate(:,gear_index);
vehiclestate_driving_mode = vehiclestate(:,driving_mode_index);
vehiclestate_station_error = vehiclestate(:,station_error_index);
vehiclestate_speed_error = vehiclestate(:,speed_error_index);
vehiclestate_lateral_error = vehiclestate(:,lateral_error_index);
vehiclestate_heading_error = vehiclestate(:,heading_error_index);
vehiclestate_control_iteration_cost_ms = vehiclestate(:,control_iteration_cost_ms_index);
%% 
speed_error_threshold = max(speed_error_threshold,0.1*max(vehiclestate_localization_velocity));
start_valid_data_index = 1;
for i = 1:vehiclestate_valid_length
    if vehiclestate_driving_mode(i) == 1  && abs(vehiclestate_speed_error(i)) < speed_error_threshold && abs(vehiclestate_lateral_error(i)) < 0.2
        start_valid_data_index = i;
        break;
    end
end

% find the end valid data index
end_valid_data_index = start_valid_data_index + 1;
for i = vehiclestate_valid_length : -1 : start_valid_data_index
    if vehiclestate_driving_mode(i) == 1 
        end_valid_data_index = max(i - 100 ,start_valid_data_index + 1);
        break;
    end
end

if end_valid_data_index - start_valid_data_index < 100
    disp("not enough valid data, not process data");
    disp("return");
    return
end
%% data screen
raw_driving_station_error =vehiclestate_station_error(start_valid_data_index:end_valid_data_index);
raw_driving_speed_error = vehiclestate_speed_error(start_valid_data_index:end_valid_data_index);
raw_driving_lateral_error = vehiclestate_lateral_error(start_valid_data_index:end_valid_data_index);
raw_driving_heading_error = vehiclestate_heading_error(start_valid_data_index:end_valid_data_index);
raw_driving_control_iteration_cost_ms = vehiclestate_control_iteration_cost_ms(start_valid_data_index:end_valid_data_index);

raw_valid_data_length = length(raw_driving_station_error);

% delete untrustable data in the process
% 1) change driving mode
% 2) wierd data like : stationg error is over 10 m 
driving_station_error = [];
driving_speed_error = [];
driving_lateral_error = [];
driving_heading_error = [];
driving_control_iteration_cost_ms = [];
flag_back_to_auto = 0;
for i =1:raw_valid_data_length
    if vehiclestate_driving_mode(start_valid_data_index+i) == 0 
        flag_back_to_auto = 0;
    elseif flag_back_to_auto == 0 && raw_driving_speed_error(i) < 0.5 
        flag_back_to_auto = 1;
    end
    next_notdriving_vector = find(vehiclestate_driving_mode(start_valid_data_index+i+1:end)==0);

    if flag_back_to_auto == 1 
        if isempty(next_notdriving_vector) || (~isempty(next_notdriving_vector) && next_notdriving_vector(1) > 100) 
            if abs(raw_driving_lateral_error(i)) < 1
                driving_lateral_error = [driving_lateral_error ;raw_driving_lateral_error(i)];
            end
            if abs(raw_driving_heading_error(i)) < 0.5
                driving_heading_error = [driving_heading_error ;raw_driving_heading_error(i)];
            end
        
            if abs(raw_driving_station_error(i)) < 5
                driving_station_error = [driving_station_error ;raw_driving_station_error(i)];
            end
            driving_speed_error = [driving_speed_error ;raw_driving_speed_error(i)];
            driving_control_iteration_cost_ms = [driving_control_iteration_cost_ms;raw_driving_control_iteration_cost_ms(i)];
        end
    end
end
%%
if length(driving_station_error)<2 || length(driving_speed_error)<2 || length(driving_lateral_error)<2 || length(driving_heading_error)<2
    disp("not enough driving data, not calculate");
    disp("return");
    return
end
%% data print
% station error
max_station_error = max(abs(driving_station_error));
station_error_tp99 = f_TP_ratio_value(abs(driving_station_error),0.99);
station_error_tp999 = f_TP_ratio_value(abs(driving_station_error),0.999);
min_station_error = min(abs(driving_station_error));
mean_station_error = mean(abs(driving_station_error));
var_station_error = std(abs(driving_station_error));
% speed error
max_speed_error = max(abs(driving_speed_error));
speed_error_tp99 = f_TP_ratio_value(abs(driving_speed_error),0.99);
speed_error_tp999 = f_TP_ratio_value(abs(driving_speed_error),0.999);
min_speed_error = min(abs(driving_speed_error));
mean_speed_error = mean(abs(driving_speed_error));
var_speed_error = std(abs(driving_speed_error));
% lateral error
max_lateral_error = max(abs(driving_lateral_error));
lateral_error_tp99 = f_TP_ratio_value(abs(driving_lateral_error),0.99);
lateral_error_tp999 = f_TP_ratio_value(abs(driving_lateral_error),0.999);
min_lateral_error = min(abs(driving_lateral_error));
mean_lateral_error = mean(abs(driving_lateral_error));
var_lateral_error = std(abs(driving_lateral_error));
% heading error
max_heading_error = max(abs(driving_heading_error))*180/pi;
heading_error_tp99 = f_TP_ratio_value(abs(driving_heading_error),0.99)*180/pi;
heading_error_tp999 = f_TP_ratio_value(abs(driving_heading_error),0.999)*180/pi;
min_heading_error = min(abs(driving_heading_error))*180/pi;
mean_heading_error = mean(abs(driving_heading_error))*180/pi;
var_heading_error = std(abs(driving_heading_error))*180/pi;
% control cost time
max_control_iteration_cost_ms = max(driving_control_iteration_cost_ms);
control_iteration_cost_ms_tp99 = f_TP_ratio_value(driving_control_iteration_cost_ms,0.99);
control_iteration_cost_ms_tp999 = f_TP_ratio_value(driving_control_iteration_cost_ms,0.999);
min_control_iteration_cost_ms = min(driving_control_iteration_cost_ms);
mean_control_iteration_cost_ms = mean(driving_control_iteration_cost_ms);
var_control_iteration_cost_ms = std(driving_control_iteration_cost_ms);

index_list = ["station_error(m)";"speed_error(m/s)";"lateral_error(m)";"heading_error(deg)";"control_cost(ms)"];
max_value_list = [max_station_error;max_speed_error;max_lateral_error;max_heading_error;max_control_iteration_cost_ms];
TP99_list = [station_error_tp99;speed_error_tp99;lateral_error_tp99;heading_error_tp99;control_iteration_cost_ms_tp99];
TP999_list = [station_error_tp999;speed_error_tp999;lateral_error_tp999;heading_error_tp999;control_iteration_cost_ms_tp999];
min_list = [min_station_error;min_speed_error;min_lateral_error;min_heading_error;min_control_iteration_cost_ms];
mean_list = [mean_station_error;mean_speed_error;mean_lateral_error;mean_heading_error;mean_control_iteration_cost_ms];
var_list = [var_station_error;var_speed_error;var_lateral_error;var_heading_error;var_control_iteration_cost_ms];
datatable = table(index_list,max_value_list,TP999_list,TP99_list,min_list,mean_list,var_list,'VariableNames',{'info','max_error','TP999','TP99','min','mean','variance'});
disp(datatable);
end