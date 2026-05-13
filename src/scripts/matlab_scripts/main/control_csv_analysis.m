%% control csv file analysis
%% JIDU Auto PnC
% liang.zhu@jiduauto.com
%%
readinfo
%%
auto_driving_list = find(vehiclestate_driving_mode==1);
if length(auto_driving_list)<50
    disp("control_csv_analysis: not enough auto process --return--");
    return
end
%% planning evaluate
fprintf("trajectory analysis: " + trajectory_filename(end-15:end-4) + "\n");
[abnormal_index_list] = f_planning_evaluate(trajectory);
if ~isempty(abnormal_index_list)
    fprintf("abnormal index:\n");
    disp(abnormal_index_list');
else
    fprintf("no obviously abnormal trajectory \n");
end
%% control error analysis
fprintf("control analysis: " + control_filename(end-15:end-4) + "\n");
f_control_evaluate(vehiclestate);
%% localization analysis
% [expected_x,expected_y,~,~] = f_localization_diagnosis(vehiclestate_x,vehiclestate_y,vehiclestate_heading,vehiclestate_localization_velocity,vehiclestate_gear);
%% diagnosis analysis
f_diagnosis_code(vehiclestate_chassis_check_code,vehiclestate_localization_check_code,vehiclestate_trajectory_check_code,vehiclestate_driving_mode)
if enable_plot ==1
    f_plot_diagnosis_code(vehiclestate);
end
%% performance plot
if enable_plot ==1
    control_analysis_plot
end
%% stop evaluation and plot stop info
if is_stop_analysis_needed==1
    stop_evaluate
end
%% steering analysis
if is_steer_analysis_needed ==1
f_analysis_steer(vehiclestate,trajectory);
end
