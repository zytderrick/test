%% decode diagnosis code 
function f_diagnosis_code(chassis_check_code,localization_check_code,trajectory_check_code,driving_mode)
% diagnosis code bit
load("common.mat");
%% delete manual driving
auto_driving_list = find(driving_mode==1);
if length(auto_driving_list)<100
    disp("no auto process, not analyze diagnosis code")
    disp("f_diagnosis_code:return");
    return
end
chassis_check_code = chassis_check_code(auto_driving_list);
trajectory_check_code = trajectory_check_code(auto_driving_list);
localization_check_code = localization_check_code(auto_driving_list);
    %% dec code to bin code
    data_length = length(chassis_check_code);
    chassis_diagnosis_count = zeros(32,2);
    localization_diagnosis_count = zeros(32,2);
    trajectory_diagnosis_count = zeros(32,2);
    total_abnormal_count = 0; 
    for i = 1:data_length
        chassis_check_code_vector = f_dec2bin_vector(chassis_check_code(i));
        localization_check_code_vector = f_dec2bin_vector(localization_check_code(i));
        trajectory_check_code_vector = f_dec2bin_vector(trajectory_check_code(i));
        chassis_diagnosis_count(:,1) = chassis_diagnosis_count(:,1) + chassis_check_code_vector;
        localization_diagnosis_count(:,1) = localization_diagnosis_count(:,1) + localization_check_code_vector;
        trajectory_diagnosis_count(:,1) = trajectory_diagnosis_count(:,1) + trajectory_check_code_vector;
        if chassis_check_code_vector(chassis_summary_bit) == 1 || localization_check_code_vector(localization_summary_bit) == 1 ...
                || trajectory_check_code_vector(trajectory_summary_bit) == 1
            total_abnormal_count = total_abnormal_count +1;
        end
    end
    chassis_diagnosis_count(:,2) = chassis_diagnosis_count(:,1) ./ data_length .*100;
    localization_diagnosis_count(:,2) = localization_diagnosis_count(:,1) ./ data_length .*100;
    trajectory_diagnosis_count(:,2) = trajectory_diagnosis_count(:,1) ./ data_length .*100;
    % chassis
    dist = struct('chassis_nulptr',chassis_diagnosis_count(chassis_nullptr_bit,:));
    dist.chassis_header = chassis_diagnosis_count(chassis_header_bit,:);
    dist.chassis_timestamp = chassis_diagnosis_count(chassis_timestamp_bit,:);
    dist.chassis_gear = chassis_diagnosis_count(chassis_gear_bit,:);
    dist.chassis_speed = chassis_diagnosis_count(chassis_speed_bit,:);
    dist.chassis_steer = chassis_diagnosis_count(chassis_steer_bit,:);
    dist.chassis_driving_mode = chassis_diagnosis_count(chassis_driving_mode_bit,:);
    dist.chassis_summary = chassis_diagnosis_count(chassis_summary_bit,:);
    dist.chassis_unknown = chassis_diagnosis_count(chassis_unknown_bit,:);
    % localization
    dist.localization_nullptr = localization_diagnosis_count(localization_nullptr_bit,:);
    dist.localization_header = localization_diagnosis_count(localization_header_bit,:);
    dist.localization_timestamp = localization_diagnosis_count(localization_timestamp_bit,:);
    dist.localization_distance = localization_diagnosis_count(localization_distance_bit,:);
    dist.localization_jump = localization_diagnosis_count(localization_jump_bit,:);
    dist.localization_status = localization_diagnosis_count(localization_status_bit,:);
    dist.localization_pose = localization_diagnosis_count(localization_pose_bit,:);
    dist.localization_continuous_abnormal = localization_diagnosis_count(localization_continuous_abnormal_bit,:);
    dist.localization_summary = localization_diagnosis_count(localization_summary_bit,:);
    dist.localization_unknown = localization_diagnosis_count(localization_unknown_bit,:);
    % trajectory
    dist.trajectory_nullptr = trajectory_diagnosis_count(trajectory_nullptr_bit,:);
    dist.trajectory_header = trajectory_diagnosis_count(trajectory_header_bit,:);
    dist.trajectory_timestamp = trajectory_diagnosis_count(trajectory_timestamp_bit,:);
    dist.trajectory_gear = trajectory_diagnosis_count(trajectory_gear_bit,:);
    dist.trajectory_point_size = trajectory_diagnosis_count(trajectory_point_size_bit,:);
    dist.trajectory_relativetime = trajectory_diagnosis_count(trajectory_relativetime_bit,:);
    dist.trajectory_kappa = trajectory_diagnosis_count(trajectory_kappa_bit,:);
    dist.trajectory_dkappa = trajectory_diagnosis_count(trajectory_dkappa_bit,:);
    dist.trajectory_estop = trajectory_diagnosis_count(trajectory_estop_bit,:);
    dist.trajectory_theta = trajectory_diagnosis_count(trajectory_theta_bit,:);
    dist.trajectory_longitudinal = trajectory_diagnosis_count(trajectory_longitudinal_bit,:);
    dist.trajectory_summary = trajectory_diagnosis_count(trajectory_summary_bit,:);
    dist.trajectory_unknown = trajectory_diagnosis_count(trajectory_unknown_bit,:);
% display diagnosis table
    result_list = [];   
    count_list = [];
    percentage_list = [];
    fields = fieldnames(dist);
    for i =1:numel(fields)
        diagnosis_char_vector = cell2mat(fields(i));
        if diagnosis_char_vector(end-6:end) == 'summary'
            continue;
        end
        if dist.(diagnosis_char_vector)(1,1)>0
            result_list = [result_list ; string(diagnosis_char_vector)];
            count_list = [count_list ; dist.(diagnosis_char_vector)(1,1)];
            percentage_list = [percentage_list ; dist.(diagnosis_char_vector)(1,2)];
        end
    end
    if length(result_list) < 1
        fprintf("No diagnosis during auto driving!\n")
        return
    end
    result_list = [result_list;'total'];
    count_list = [count_list ; total_abnormal_count];
    percentage_list = [percentage_list ; total_abnormal_count/data_length*100];
    diagnosis_table = table(result_list,count_list,percentage_list,'VariableNames',{'diagnosis result','count','percentage(%)'});
    disp(diagnosis_table);
    
end
