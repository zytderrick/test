%% plot diagnosis code 
function f_plot_diagnosis_code(vehiclestate)
%%
load("common.mat");
%%
last_zero_index = 0;
for i =1:length(vehiclestate)
    if vehiclestate(i,1)>1e-1
        break;
    else
        last_zero_index = last_zero_index +1 ;
    end
end
start_index = last_zero_index +1;
vehiclestate_timestamp = vehiclestate(start_index:end,timestamp_sec_index);
vehiclestate_chassis_check_code = vehiclestate(start_index:end,chassis_check_code_index);
vehiclestate_localization_check_code = vehiclestate(start_index:end,localization_check_code_index);
vehiclestate_trajectory_check_code = vehiclestate(start_index:end,trajectory_check_code_index);
%%

vehiclestate_plot_time = vehiclestate_timestamp(:,1) - vehiclestate_timestamp(1);
data_length = length(vehiclestate_timestamp);

chassis_diagnosis_code_matrix = [];
trajectory_diagnosis_code_matrix = [];
localization_diagnosis_code_matrix = [];
weight_vector = 1:32;
weight_vector = weight_vector';
for i =1:data_length
    chassis_check_code_vector = f_dec2bin_vector(vehiclestate_chassis_check_code(i));
    localization_check_code_vector = f_dec2bin_vector(vehiclestate_localization_check_code(i));
    trajectory_check_code_vector = f_dec2bin_vector(vehiclestate_trajectory_check_code(i));
    chassis_check_code_vector_weighted = chassis_check_code_vector.*weight_vector;
    localization_check_code_vector_weighted = localization_check_code_vector.*weight_vector;
    trajectory_check_code_vector_weighted = trajectory_check_code_vector.*weight_vector;
    chassis_diagnosis_code_matrix = [chassis_diagnosis_code_matrix,chassis_check_code_vector_weighted];
    trajectory_diagnosis_code_matrix = [trajectory_diagnosis_code_matrix,trajectory_check_code_vector_weighted];
    localization_diagnosis_code_matrix = [localization_diagnosis_code_matrix,localization_check_code_vector_weighted];
end
figure(60)
subplot(3,1,1)
for i =1:32
    plot(vehiclestate_plot_time,chassis_diagnosis_code_matrix(i,:),'r.');hold on
end
xlabel("time[s]","FontSize",20);
ylabel("chassis diagnosis bit","FontSize",20);
subplot(3,1,2)
for i =1:32
    plot(vehiclestate_plot_time,trajectory_diagnosis_code_matrix(i,:),'b.');hold on
end
xlabel("time[s]","FontSize",20);
ylabel("trajectory diagnosis bit","FontSize",20);
subplot(3,1,3)
for i =1:32
    plot(vehiclestate_plot_time,localization_diagnosis_code_matrix(i,:),'g.');hold on
end
xlabel("time[s]","FontSize",20);
ylabel("localization diagnosis bit","FontSize",20);
end