%% read trajecory and control info
%% JIDU Auto PnC
% liang.zhu@jiduauto.com
trajectory = readmatrix(trajectory_filename);
vehiclestate = readmatrix(control_filename);
%% trajectory info 
trajectory_total_length = size(trajectory,1)-1;
trajectory_index = trajectory(1:end-1,trajectory_index_index);
trajectory_x = trajectory(1:end-1,trajectory_x_index);
trajectory_y = trajectory(1:end-1,trajectory_y_index);
trajectory_theta = trajectory(1:end-1,trajectory_theta_index);
trajectory_kappa= trajectory(1:end-1,trajectory_kappa_index);
trajectory_dkappa = trajectory(1:end-1,trajectory_dkappa_index);
trajectory_v = trajectory(1:end-1,trajectory_v_index);
trajectory_a = trajectory(1:end-1,trajectory_a_index);
trajectory_relativetime = trajectory(1:end-1,trajectory_relativetime_index);
trajectory_headertime = trajectory(1:end-1,trajectory_headertime_index);
trajectory_s = trajectory(1:end-1,trajectory_s_index);
trajectory_gear = trajectory(1:end-1,trajectory_gear_index);
trajectory_replan = trajectory(1:end-1,trajectory_replan_index);
%% vehiclestate info
last_zero_index = 0;
for i =1:size(vehiclestate,1)
    if vehiclestate(i,1)>1e-1
        break;
    else
        last_zero_index = last_zero_index +1 ;
    end
end
start_index = last_zero_index +1;
vehiclestate_total_length = size(vehiclestate,1);
vehiclestate_valid_length = vehiclestate_total_length - last_zero_index;

vehiclestate_timestamp = vehiclestate(start_index:end,timestamp_sec_index);
vehiclestate_x = vehiclestate(start_index:end,raw_x_index);
vehiclestate_y = vehiclestate(start_index:end,raw_y_index);
vehiclestate_heading = vehiclestate(start_index:end,raw_heading_index);
vehiclestate_localization_velocity = vehiclestate(start_index:end,localization_velocity_index);
vehiclestate_linear_velocity = vehiclestate(start_index:end,linear_velocity_index);
vehiclestate_linear_acceleration = vehiclestate(start_index:end,linear_acceleration_index);
vehiclestate_steering_angle = vehiclestate(start_index:end,steering_angle_index);
vehiclestate_steering_angle_spd = vehiclestate(start_index:end,steering_angle_spd_index);
vehiclestate_gear = vehiclestate(start_index:end,gear_index);
vehiclestate_driving_mode = vehiclestate(start_index:end,driving_mode_index);
vehiclestate_acceleration_cmd = vehiclestate(start_index:end,acceleration_cmd_index);
vehiclestate_steering_cmd = vehiclestate(start_index:end,steering_cmd_index);
vehiclestate_station_error = vehiclestate(start_index:end,station_error_index);
vehiclestate_speed_error = vehiclestate(start_index:end,speed_error_index);
vehiclestate_lateral_error = vehiclestate(start_index:end,lateral_error_index);
vehiclestate_heading_error = vehiclestate(start_index:end,heading_error_index);
vehiclestate_control_iteration_cost_ms = vehiclestate(start_index:end,control_iteration_cost_ms_index);
vehiclestate_localization_jump_count = vehiclestate(start_index:end,localization_jump_count_index);
vehiclestate_fallback_reaction_index = vehiclestate(start_index:end,fallback_reaction_index_index);
vehiclestate_pre_process_cost_ms = vehiclestate(start_index:end,pre_process_cost_ms_index);
vehiclestate_produce_cmd_cost_ms = vehiclestate(start_index:end,produce_cmd_cost_ms_index);
vehiclestate_post_process_cost_ms = vehiclestate(start_index:end,post_process_cost_ms_index);
vehiclestate_chassis_check_code = vehiclestate(start_index:end,chassis_check_code_index);
vehiclestate_localization_check_code = vehiclestate(start_index:end,localization_check_code_index);
vehiclestate_trajectory_check_code = vehiclestate(start_index:end,trajectory_check_code_index);
vehiclestate_accumulated_error = vehiclestate(start_index:end,accumulated_error_index);
vehiclestate_standstill = vehiclestate(start_index:end,standstill_index);
vehiclestate_require_standstill_steering = vehiclestate(start_index:end,require_standstill_steering_index);


