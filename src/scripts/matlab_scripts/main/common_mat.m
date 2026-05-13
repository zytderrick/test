%% save common infomation in the .mat
%% JIDU Auto PnC
% liang.zhu@jiduauto.com
%% vehicle param
transmission_ratio = 14.6;
L = 2.999;
max_SWA = 457;
max_SWA_rate = 400;
%% control conf
trajectorylog_control_period = 20;
control_ts = 0.02;
kappa_limit = tan(max_SWA*pi/(180*transmission_ratio))/L;
dkappa_limit = max_SWA_rate*pi/(180*L*transmission_ratio);
a_ub = 2;
a_lb = -6;
%% planning conf
planning_d_relavtive_time = 0.05;
%% index for csv
% vehiclestate csv
timestamp_sec_index = 1;
raw_x_index = 2;
raw_y_index = 3;
raw_z_index = 4;
raw_roll_index = 5;
raw_pitch_index = 6;
raw_yaw_index = 7;
raw_heading_index = 8;
curvature_index = 9;
angular_velocity_index = 10;
speed_x_index = 11;
speed_y_index = 12;
speed_z_index = 13;
localization_velocity_index = 14;
linear_velocity_index = 15;
linear_acceleration_index = 16;
steering_angle_index = 17;
steering_angle_spd_index = 18;
steering_percentage_index = 19;
gear_index = 20; 
driving_mode_index = 21;
driving_mode_cmd_index = 22; 
gear_cmd_index = 23;
acceleration_cmd_index = 24;
steering_cmd_index = 25;
steering_rate_cmd_index = 26;
driving_action_index = 27;
station_error_index = 28;
speed_error_index = 29;
lateral_error_index = 30;
heading_error_index = 31;
control_iteration_cost_ms_index = 32;
localization_jump_count_index = 33;
count_bad_kappa_index = 34;
count_bad_dkappa_index = 35;
fallback_reaction_index_index = 36;
pre_process_cost_ms_index = 40;
produce_cmd_cost_ms_index = 41;
post_process_cost_ms_index = 42;
chassis_check_code_index = 43;
localization_check_code_index = 44;
trajectory_check_code_index = 45;
pure_x_index = 46;
pure_y_index = 47;
pure_yaw_index = 48;
acc_pedal_abnormal_index = 49;
standstill_index = 50;
require_standstill_steering_index = 51;
accumulated_error_index = 52;

% trajectory csv
trajectory_index_index = 1;
trajectory_x_index = 2;
trajectory_y_index = 3;
trajectory_z_index = 4;
trajectory_theta_index = 5;
trajectory_kappa_index = 6;
trajectory_dkappa_index = 7;
trajectory_v_index = 8;
trajectory_a_index = 9;
trajectory_relativetime_index = 10;
trajectory_headertime_index = 11;
trajectory_s_index = 12;
trajectory_gear_index = 13;
trajectory_replan_index = 14;
%% valid auto process threshold
speed_error_threshold = 0.2;
lateral_error_threshold = 0.2;

%% diagnosis bit 
chassis_nullptr_bit = 1;
chassis_header_bit = 2;
chassis_timestamp_bit = 3;
chassis_timestamp_level_bit = 4;
chassis_gear_bit = 5;
chassis_gear_level_bit = 6;
chassis_speed_bit = 7;
chassis_speed_level_bit = 8;
chassis_steer_bit = 9;
chassis_steer_level_bit = 10;
chassis_driving_mode_bit = 11;
chassis_driving_mode_level_bit = 12;
chassis_summary_bit = 29;
chassis_summary_level_bit = 30;
chassis_unknown_bit = 32;

localization_nullptr_bit = 1;
localization_header_bit = 2;
localization_timestamp_bit = 3;
localization_timestamp_level_bit =4;
localization_distance_bit = 5;
localization_distance_level_bit = 6;
localization_jump_bit = 7;
localization_jump_level_bit = 8;
localization_status_bit = 9;
localization_status_level_bit = 10;
localization_pose_bit = 11;
localization_pose_level_bit = 12;
localization_continuous_abnormal_bit = 27;
localization_continuous_abnormal_level_bit = 28;
localization_summary_bit = 29;
localization_summary_level_bit = 30;
localization_unknown_bit = 32;

trajectory_nullptr_bit = 1;
trajectory_header_bit = 2;
trajectory_timestamp_bit = 3;
trajectory_timestamp_level_bit =4;
trajectory_gear_bit = 5;
trajectory_gear_level_bit = 6;
trajectory_point_size_bit = 7;
trajectory_point_size_level_bit = 8;
trajectory_relativetime_bit =9;
trajectory_relativetime_level_bit = 10;
trajectory_kappa_bit = 11;
trajectory_kappa_level_bit = 12;
trajectory_dkappa_bit = 13;
trajectory_dkappa_level_bit = 14;
trajectory_estop_bit = 15;
trajectory_estop_level_bit = 16;
trajectory_theta_bit = 17;
trajectory_theta_level_bit = 18;
trajectory_longitudinal_bit = 19;
trajectory_longitudinal_level_bit = 20;
trajectory_summary_bit = 29;
trajectory_summary_level_bit = 30;
trajectory_unknown_bit = 32;
%% function switch
is_stop_analysis_needed = 1;
is_steer_analysis_needed = 0;
batch_data_analysis = 1;
%% plot switch
enable_plot = 1;
% if analyzing batch data, disable plot
enable_plot = enable_plot * ~logical(batch_data_analysis);
enable_plot_vector = ones(22,1);
if enable_plot == 0
    enable_plot_vector = zeros(22,1);
end

enable_plot_traj_xy = 1 * enable_plot_vector(1);
enable_plot_traj_theta = 0 * enable_plot_vector(2);
enable_plot_traj_v = 1 * enable_plot_vector(3);
enable_plot_traj_a = 0 * enable_plot_vector(4);
enable_plot_traj_s = 1 * enable_plot_vector(5);
enable_plot_traj_kappa = 0 * enable_plot_vector(6);
enable_plot_traj_distance = 0 * enable_plot_vector(7);
enable_plot_localization = 1 * enable_plot_vector(8);
enable_plot_vehiclestate_distance = 0 * enable_plot_vector(9);
enable_plot_heading = 0 * enable_plot_vector(10);
enable_plot_lon = 1 * enable_plot_vector(11);
enable_plot_lat = 0 * enable_plot_vector(12);
enable_plot_theta_steercmd = 0 * enable_plot_vector(13);
enable_plot_traj_v_acccmd = 0 * enable_plot_vector(14);
enable_plot_lat_err = 1 * enable_plot_vector(15);
enable_plot_lon_err = 1 * enable_plot_vector(16);
enable_plot_lon_diff = 0 * enable_plot_vector(17);
enable_plot_lat_diff = 0 * enable_plot_vector(18);
enable_plot_steer_cmd_feedback = 0 * enable_plot_vector(19);
enable_plot_speed_localization_chassis = 1 * enable_plot_vector(20);
enable_plot_cost_time = 0 * enable_plot_vector(21);
enable_plot_accumulated_error = 0 * enable_plot_vector(22);

%% planning evaluate switch
enable_singleframe_v_a_kappa_check = 0;
enable_interframe_diff_check = 0;
enable_singleframe_kinetic_check = 0;
enable_singleframe_v2xy_check = 0;
enable_singleframe_xy2theta_check = 0;
enable_singleframe_v_a_s_check = 0;
%% trajectory length
% set number of points to plot at each instance 1~101
number_plot_point_each_instance = 50;
%%
mat_file_name = "common.mat";
save(mat_file_name)