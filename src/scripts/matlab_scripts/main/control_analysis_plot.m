%% plot trajectory ,control, trajectory vs control
%% 
load("common.mat");
%% save picture folder
% get time string
datestring = datestr(datetime);
timestr = datestring(end-7:end);
foldername = "pngfile_"+ datestring;
mkdir(foldername);
cd(foldername)
%% process data
color_list = ['r','g','b','c','m','y','k'];
color_size = size(color_list,2);
marker_list = ['o','x','.','*'];
marker_size = size(marker_list,2);
for i = 1 :trajectory_total_length
    if trajectory_relativetime(i) >100
        trajectory_relativetime(i) = 0;
    end
end
trajectory_plot_time = trajectory_headertime(:,1) - trajectory_headertime(1);
trajectory_plot_time = trajectory_plot_time + trajectory_relativetime;
trajectory_theta_deg = trajectory_theta .*180/pi;
trajectory_interpoint_distatnce = zeros(trajectory_total_length,1);
trajectory_accumulated_distatnce = zeros(trajectory_total_length,1);
trajectory_calculated_theta_deg = zeros(trajectory_total_length,1);
trajectory_ds = zeros(trajectory_total_length,1);
trajectory_speed2ds = zeros(trajectory_total_length,1);
for i =1:trajectory_total_length -1
    if trajectory_index(i)== trajectory_index(i+1)
        current_trajectory_pos = [trajectory_x(i),trajectory_y(i)];
        next_trajectory_pos = [trajectory_x(i+1),trajectory_y(i+1)];
        trajectory_interpoint_distatnce(i) = norm(next_trajectory_pos-current_trajectory_pos);
        trajectory_dx = trajectory_x(i+1) - trajectory_x(i);
        trajectory_dy = trajectory_y(i+1) - trajectory_y(i);
        calculated_theta_deg = atan2(trajectory_dy,trajectory_dx) *180/pi;
        trajectory_calculated_theta_deg(i) = calculated_theta_deg;
        trajectory_ds(i) = trajectory_s(i+1) - trajectory_s(i);
        trajectory_speed2ds(i) = mean([trajectory_v(i),trajectory_v(i+1)])*(trajectory_plot_time(i+1)-trajectory_plot_time(i));
    else
        trajectory_calculated_theta_deg(i) = trajectory_theta_deg(i);
    end
    if i>1 && trajectory_index(i) == trajectory_index(i-1)
        trajectory_accumulated_distatnce(i) = trajectory_accumulated_distatnce(i-1) + trajectory_interpoint_distatnce(i);
    end
end
trajectory_delta_theta_deg = zeros(trajectory_total_length,1);
for i =1:trajectory_total_length -1
    trajectory_delta_theta_deg(i) = trajectory_theta_deg(i+1) - trajectory_theta_deg(i);
end
%% interpoint_distance
vehiclestate_plot_time = vehiclestate_timestamp(:,1) - vehiclestate_timestamp(1);
vehiclestate_heading_deg = vehiclestate_heading.*180/pi;    
%distance、lateral_error、heading_error
vehiclestate_interpoint_distance = zeros(vehiclestate_valid_length,1);
vehiclestate_lcoalization_s = zeros(vehiclestate_valid_length,1);
vehiclestate_chassis_v2s = zeros(vehiclestate_valid_length,1);
vehiclestate_localization_v2s = zeros(vehiclestate_valid_length,1);
vehiclestate_lcoalization_s_in_windowsize = zeros(vehiclestate_valid_length,1);
vehiclestate_chassis_v2s_in_windowsize = zeros(vehiclestate_valid_length,1);
vehiclestate_localization_v2s_in_windowsize = zeros(vehiclestate_valid_length,1);
for i =2:vehiclestate_valid_length
    current_pos = [vehiclestate_x(i-1),vehiclestate_y(i-1)];
    next_pos = [vehiclestate_x(i),vehiclestate_y(i)];
    vehiclestate_interpoint_distance(i) = norm(next_pos-current_pos);
    last_localzation_v2s = mean([vehiclestate_localization_velocity(i-1);vehiclestate_localization_velocity(i)])*(vehiclestate_timestamp(i)-vehiclestate_timestamp(i-1));
    last_chassis_v2s = mean([vehiclestate_linear_velocity(i-1);vehiclestate_linear_velocity(i)])*(vehiclestate_timestamp(i)-vehiclestate_timestamp(i-1));
    if vehiclestate_interpoint_distance(i)>0.6
        vehiclestate_interpoint_distance(i)=0;
    end
    vehiclestate_lcoalization_s(i) = vehiclestate_lcoalization_s(i-1) + vehiclestate_interpoint_distance(i);
    vehiclestate_chassis_v2s(i) = vehiclestate_chassis_v2s(i-1) + vehiclestate_localization_velocity(i-1)*control_ts;
    vehiclestate_localization_v2s(i) = vehiclestate_localization_v2s(i-1) + last_localzation_v2s;
end
calculated_s_windowsize = 100;
for i =calculated_s_windowsize +1 :vehiclestate_valid_length
    vehiclestate_lcoalization_s_in_windowsize(i) = vehiclestate_lcoalization_s(i) - vehiclestate_lcoalization_s(i-calculated_s_windowsize);
    vehiclestate_chassis_v2s_in_windowsize(i) = vehiclestate_chassis_v2s(i) - vehiclestate_chassis_v2s(i-calculated_s_windowsize);
    vehiclestate_localization_v2s_in_windowsize(i) = vehiclestate_localization_v2s(i) - vehiclestate_localization_v2s(i-calculated_s_windowsize);
end
localization_v2s_minus_chassis_s = vehiclestate_localization_v2s - vehiclestate_chassis_v2s;
vehiclestate_lcoalization_s_minus_chassis_s = vehiclestate_lcoalization_s - vehiclestate_chassis_v2s;
%%
delta_time_traj_vehicle = trajectory_headertime(1) - vehiclestate_timestamp(1);
%% separate driving mode
vehiclestate_driving_indexlist=[];
vehiclestate_notdriving_indexlist=[];
soft_brake_indexlist = [];
deceleration_indexlist = [];
localization_jump_indexlist = [];
gear_reverse_list = [];
for i =1:vehiclestate_valid_length
    if vehiclestate_driving_mode(i) == 1 
        vehiclestate_driving_indexlist= [vehiclestate_driving_indexlist;i];
    else
        vehiclestate_notdriving_indexlist = [vehiclestate_notdriving_indexlist;i];
    end
    if vehiclestate_driving_mode(i) == 1 && vehiclestate_fallback_reaction_index(i) == 2
            soft_brake_indexlist=[soft_brake_indexlist;i];
    end
    if vehiclestate_driving_mode(i) == 1 && vehiclestate_acceleration_cmd(i) <-0.05 && vehiclestate_acceleration_cmd(i) >-0.8
            deceleration_indexlist=[deceleration_indexlist;i];
    end
    if vehiclestate_driving_mode(i) == 1 && vehiclestate_gear(i) == 2
        gear_reverse_list = [gear_reverse_list;i];
    end
    if i>1 && vehiclestate_driving_mode(i) == 1 && vehiclestate_localization_jump_count(i)> vehiclestate_localization_jump_count(i-1)
            localization_jump_indexlist=[localization_jump_indexlist;i];
    end
end
%% plot trajectory
trajectory_plot_range = 1:max(trajectory_index);
% trajectory_plot_range = trajectory_plot_range +1 ;
% fprintf("index:  "+trajectory_plot_range+"  size:  "+length(find(trajectory_index==trajectory_plot_range))+"\n")
% trajectory_plot_range = abnormal_index_list';
plot_trajectory
%% plot localization info
if enable_plot_localization ==1
    figure(8)
    plot(vehiclestate_x,vehiclestate_y,'.',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r","MarkerSize",5);hold on
    plot(vehiclestate_x,vehiclestate_y,'.',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b","MarkerSize",5);hold on
    plot(vehiclestate_x,vehiclestate_y,'.',"MarkerIndices",gear_reverse_list,"MarkerEdgeColor",[1 0.8 0],"MarkerSize",5);hold on
    plot(vehiclestate_x,vehiclestate_y,'.',"MarkerIndices",soft_brake_indexlist,"Marker","o","MarkerEdgeColor","g","MarkerSize",20);hold on
    plot(vehiclestate_x(vehiclestate_driving_indexlist(1)),vehiclestate_y(vehiclestate_driving_indexlist(1)),"Marker","^","MarkerSize",15,"LineWidth",3)
    plot(vehiclestate_x,vehiclestate_y,'+',"MarkerIndices",localization_jump_indexlist,"MarkerEdgeColor","k","MarkerSize",20,"LineWidth",3);hold on
    xlabel("x[m]","FontSize",20)
    ylabel("y[m]","FontSize",20)
    axis equal
    grid on
    legend("auto","mannual","gear reverse","soft brake","start point","localization jump","FontSize",20)
    title("localization","FontSize",25)
    xlim([min(vehiclestate_x(vehiclestate_driving_indexlist))-10 max(vehiclestate_x(vehiclestate_driving_indexlist))+10])
    ylim([min(vehiclestate_y(vehiclestate_driving_indexlist))-10 max(vehiclestate_y(vehiclestate_driving_indexlist))+10])
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "vehiclestate_xy"+ timestr + ".png";
    saveas(gcf,save_file_name)
end

if enable_plot_vehiclestate_distance ==1
    figure(9)
    subplot(3,1,1)
    plot(vehiclestate_plot_time,vehiclestate_interpoint_distance,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_interpoint_distance,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    xlabel("time[s]","FontSize",20)
    ylabel("interpoint distance[m]","FontSize",20)
    grid on
    title("distance","FontSize",25)
    legend("auto","manual","Fontsize",20)
    ylim([0 0.2])
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])

    subplot(3,1,2)
    plot(vehiclestate_plot_time,vehiclestate_lcoalization_s,'ro',"MarkerSize",3);hold on
    plot(vehiclestate_plot_time,vehiclestate_chassis_v2s,'bo',"MarkerSize",3);hold on
    plot(vehiclestate_plot_time,vehiclestate_localization_v2s,'color','k');hold on
    xlabel("time[s]","FontSize",20)
    ylabel("accumulated distance[m]","FontSize",20)
    grid on
    title("distance","FontSize",25)
    legend("localization s","chassis s","localization v2s","Fontsize",20)
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    
    subplot(3,1,3)
    plot(vehiclestate_plot_time,vehiclestate_lcoalization_s_in_windowsize,'ro',"MarkerSize",3);hold on
    plot(vehiclestate_plot_time,vehiclestate_chassis_v2s_in_windowsize,'bo',"MarkerSize",3);hold on
    plot(vehiclestate_plot_time,vehiclestate_localization_v2s_in_windowsize,'color','k');hold on
    legend("localization s","chassis s","localization v2s","Fontsize",20)
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "vehiclestate_distance"+ timestr + ".png";
    saveas(gcf,save_file_name)
end

if enable_plot_heading ==1
    figure(10)
    subplot(2,1,1)
    plot(vehiclestate_plot_time,vehiclestate_driving_mode,'Marker','o','Markersize',1,'Color','b'); hold on 
    plot(vehiclestate_plot_time,vehiclestate_gear,'Marker','o','Markersize',1,'Color','r'); hold on 
    xlabel("time[s]","FontSize",20)
    ylabel("driving mode","FontSize",20)
    xlim([0 max(vehiclestate_plot_time)])
    hold on

    subplot(2,1,2)
    plot(vehiclestate_plot_time,vehiclestate_heading_deg,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_heading_deg,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("heading[deg]","FontSize",20)
    grid on
    title("localization","FontSize",25)
    ylim([-180 180])
    xlim([0 max(vehiclestate_plot_time)])
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "vehiclestate_heading"+ timestr + ".png";
    saveas(gcf,save_file_name)
end
%% plot 11
%acc_cmd,v,a
if enable_plot_lon ==1
    figure(11)
    subplot(4,1,1)
    plot(vehiclestate_plot_time,vehiclestate_driving_mode,'Marker','o','Markersize',1,'Color','b'); hold on 
    plot(vehiclestate_plot_time,vehiclestate_gear,'Marker','o','Markersize',1,'Color','r'); hold on 
    legend("driving mode","gear")
    xlabel("time[s]","FontSize",20)
    ylabel("driving mode","FontSize",20)
    xlim([0 max(vehiclestate_plot_time)])
    hold on
    
    subplot(4,1,2)
    plot(vehiclestate_plot_time,vehiclestate_acceleration_cmd,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_acceleration_cmd,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("acceleration cmd[m/s^2]","FontSize",20)
    grid on
    xlim([0 max(vehiclestate_plot_time)])
    ylim([-0.8 max(min(max(vehiclestate_acceleration_cmd),5),-0.6)])
    
    hold on 
    subplot(4,1,3)
    plot(vehiclestate_plot_time,vehiclestate_linear_velocity,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_linear_velocity,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("linear velocity[m/s]","FontSize",20)
    grid on
    xlim([0 max(vehiclestate_plot_time)])
    
    hold on 
    subplot(4,1,4)
    plot(vehiclestate_plot_time,vehiclestate_linear_acceleration,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_linear_acceleration,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("linear acceleration[m/s^2]","FontSize",20)
    grid on
    xlim([0 max(vehiclestate_plot_time)])
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "vehiclestate_lon"+ timestr + ".png";
    saveas(gcf,save_file_name)
end
%% plot 12
%steer_cmd,steer_angle,steer_angle_spd
if enable_plot_lat ==1
    figure(12)
    subplot(4,1,1)
    plot(vehiclestate_plot_time,vehiclestate_driving_mode,'Marker','o','Markersize',1,'Color','b'); hold on 
    plot(vehiclestate_plot_time,vehiclestate_gear,'Marker','o','Markersize',1,'Color','r'); hold on 
    xlabel("time[s]","FontSize",20)
    ylabel("driving mode","FontSize",20)
    legend("driving mode","gear","FontSize",10)
    xlim([0 max(vehiclestate_plot_time)])
    hold on
    
    subplot(4,1,2)
    plot(vehiclestate_plot_time,vehiclestate_steering_cmd,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_steering_cmd,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("steering cmd[deg]","FontSize",20)
    grid on
    xlim([0 max(vehiclestate_plot_time)])
    hold on
    
    subplot(4,1,3)
    plot(vehiclestate_plot_time,vehiclestate_steering_angle,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_steering_angle,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("steering angle[deg]","FontSize",20)
    grid on
    xlim([0 max(vehiclestate_plot_time)])
    hold on 
    
    subplot(4,1,4)
    plot(vehiclestate_plot_time,vehiclestate_steering_angle_spd,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_steering_angle_spd,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("steering angle speed[deg]","FontSize",20)
    grid on
    xlim([0 max(vehiclestate_plot_time)])
    ylim([-2 2])
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "vehiclestate_lat"+ timestr + ".png";
    saveas(gcf,save_file_name)
end
%% plot 13
%planning theta & steer cmd 
if enable_plot_theta_steercmd ==1
    figure(13)
    subplot(3,1,1)
    plot(vehiclestate_plot_time,vehiclestate_driving_mode,'Marker','o','Markersize',1,'Color','b'); hold on 
    plot(vehiclestate_plot_time,vehiclestate_gear,'Marker','o','Markersize',1,'Color','r'); hold on 
    xlabel("time[s]","FontSize",20)
    ylabel("driving mode","FontSize",20)
    legend("driving mode","gear","Fontsize",20)
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    hold on
    
    subplot(3,1,2)
    for i = 1:max(trajectory_index)
         current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_plot_time(plot_start_index:plot_end_index) + delta_time_traj_vehicle,...
            trajectory_theta_deg(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',1,'Marker',marker_list(mod(i,marker_size)+1))  
        hold on
    end
    xlabel("time[s]","FontSize",20)
    ylabel("theta[deg]","FontSize",20)
    grid on
    ylim([-180 180])
    % ylim([min(trajectory_theta_deg) max(trajectory_theta_deg)])
    title("planning","FontSize",25)
    xlim([trajectory_plot_time(1) trajectory_plot_time(end)])    
    hold on

    subplot(3,1,3)
    plot(vehiclestate_plot_time,vehiclestate_steering_cmd,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_steering_cmd,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("steering cmd[deg]","FontSize",20)
    grid on
    title("control","FontSize",25)
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    ylim([-460 460])
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "theta&steer"+ timestr + ".png";
    saveas(gcf,save_file_name)
end
%% plot 14
%planning v, acc_cmd,v,a
if enable_plot==1 && enable_plot_traj_v_acccmd ==1
    figure(14)
    subplot(4,1,1)
    plot(vehiclestate_plot_time,vehiclestate_driving_mode,'Marker','o','Markersize',1,'Color','b'); hold on 
    plot(vehiclestate_plot_time,vehiclestate_gear,'Marker','o','Markersize',1,'Color','r'); hold on 
    xlabel("time[s]","FontSize",20)
    ylabel("driving mode","FontSize",20)
    legend("driving mode","gear","Fontsize",20)
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    hold on
    
    subplot(4,1,2)
    for i = 1:max(trajectory_index)
        current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_plot_time(plot_start_index:plot_end_index)  + delta_time_traj_vehicle,...
            trajectory_v(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',3,'Marker',marker_list(mod(i,marker_size)+1))  
        hold on
    end
    xlabel("time[s]","FontSize",20)
    ylabel("v[m/s]","FontSize",20)
    grid on
    title("planning","FontSize",25)
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    
    hold on
    subplot(4,1,3)
    plot(vehiclestate_plot_time,vehiclestate_linear_velocity,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_linear_velocity,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("linear velocity[m/s]","FontSize",20)
    grid on
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    
    hold on 
    subplot(4,1,4)
    plot(vehiclestate_plot_time,vehiclestate_acceleration_cmd,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_acceleration_cmd,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("acceleration cmd[m/s^2]","FontSize",20)
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "traj_v&acc"+ timestr + ".png";
    saveas(gcf,save_file_name)
end
%% plot 15
%lateral error & heading error & steer cmd
if enable_plot==1 && enable_plot_lat_err ==1
    figure(15)
    subplot(4,1,1)
    plot(vehiclestate_plot_time,vehiclestate_driving_mode,'Marker','o','Markersize',1,'Color','b'); hold on 
    plot(vehiclestate_plot_time,vehiclestate_gear,'Marker','o','Markersize',1,'Color','r'); hold on 
    xlabel("time[s]","FontSize",20)
    ylabel("driving mode","FontSize",20)
    legend("driving mode","gear","Fontsize",20)
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    hold on
    
    subplot(4,1,2)
    plot(vehiclestate_plot_time,vehiclestate_lateral_error,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_lateral_error,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("lateral error[m]","FontSize",20)
    grid on
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    ylim([-0.3 0.3])
    hold on
    
    subplot(4,1,3)
    plot(vehiclestate_plot_time,vehiclestate_heading_error.*180/pi,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_heading_error.*180/pi,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("heading error[deg]","FontSize",20)
    grid on
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    ylim([-20 20])
    hold on 
    
    subplot(4,1,4)
    plot(vehiclestate_plot_time,vehiclestate_steering_cmd,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_steering_cmd,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("steering cmd[deg]","FontSize",20)
    grid on
    xlim([vehiclestate_plot_time(1) max(vehiclestate_plot_time)])
    ylim([max(-460,min(vehiclestate_steering_cmd)) min(460,max(vehiclestate_steering_cmd))])
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "vehiclestate_lat_error"+ timestr + ".png";
    saveas(gcf,save_file_name)
end
%% plot 16
%station error & speed error & acc cmd
if enable_plot==1 && enable_plot_lon_err ==1
    figure(16)
    subplot(4,1,1)
    plot(vehiclestate_plot_time,vehiclestate_driving_mode,'Marker','o','Markersize',1,'Color','b'); hold on 
    plot(vehiclestate_plot_time,vehiclestate_gear,'Marker','o','Markersize',1,'Color','r'); hold on 
    xlabel("time[s]","FontSize",20)
    ylabel("driving mode","FontSize",20)
    legend("driving mode","gear","Fontsize",20)
    grid on
    xlim([0 max(vehiclestate_plot_time)])
    hold on
    
    subplot(4,1,2)
    plot(vehiclestate_plot_time,vehiclestate_station_error,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_station_error,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("station error[m]","FontSize",20)
    grid on
    xlim([0 max(vehiclestate_plot_time)])
    hold on
    
    subplot(4,1,3)
    plot(vehiclestate_plot_time,vehiclestate_speed_error,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_speed_error,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("speed error[m/s]","FontSize",20)
    grid on
    xlim([0 max(vehiclestate_plot_time)])
    hold on 
    
    subplot(4,1,4)
    plot(vehiclestate_plot_time,vehiclestate_acceleration_cmd,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_acceleration_cmd,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
    legend("auto","mannual","FontSize",10)
    xlabel("time[s]","FontSize",20)
    ylabel("acceleration cmd[m/s^2]","FontSize",20)
    grid on
    xlim([0 max(vehiclestate_plot_time)])
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "vehiclestate_lon_error"+ timestr + ".png";
    saveas(gcf,save_file_name)
end
%% d_acc,d_acc_cmd
if enable_plot_lon_diff ==1
d_acc = [0;diff(vehiclestate_linear_acceleration)];
d_acc_cmd = [0;diff(vehiclestate_acceleration_cmd)];
figure(17)
subplot(3,1,1)
plot(vehiclestate_plot_time,vehiclestate_driving_mode,'Marker','o','Markersize',1,'Color','b'); hold on 
plot(vehiclestate_plot_time,vehiclestate_gear,'Marker','o','Markersize',1,'Color','r'); hold on 
xlabel("time[s]","FontSize",20)
ylabel("driving mode","FontSize",20)
grid on
xlim([0 max(vehiclestate_plot_time)])
hold on

subplot(3,1,2)
plot(vehiclestate_plot_time,d_acc,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
plot(vehiclestate_plot_time,d_acc,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
legend("auto","mannual","FontSize",10)
xlabel("time[s]","FontSize",20)
ylabel("chassis acceleration diff[m/s^3]","FontSize",20)
grid on
xlim([0 max(vehiclestate_plot_time)])
ylim([-0.3 0.3])

subplot(3,1,3)
plot(vehiclestate_plot_time,d_acc_cmd,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
plot(vehiclestate_plot_time,d_acc_cmd,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
legend("auto","mannual","FontSize",10)
xlabel("time[s]","FontSize",20)
ylabel("acceleration cmd diff[m/s^3]","FontSize",20)
grid on
xlim([0 max(vehiclestate_plot_time)])
ylim([-0.3 0.3])
set(gcf,'Position',[0 0 2600 1800]);
save_file_name = "vehiclestate_acc_diff"+ timestr + ".png";
saveas(gcf,save_file_name)
end
%% steering_angle_spd,d_steer_cmd
if enable_plot_lat_diff ==1
d_steer_cmd = [0;diff(vehiclestate_steering_cmd)];
d_steer_angle = [0;diff(vehiclestate_steering_angle)];
figure(18)
subplot(3,1,1)
plot(vehiclestate_plot_time,vehiclestate_driving_mode,'Marker','o','Markersize',1,'Color','b'); hold on 
plot(vehiclestate_plot_time,vehiclestate_gear,'Marker','o','Markersize',1,'Color','r'); hold on 
xlabel("time[s]","FontSize",20)
ylabel("driving mode","FontSize",20)
grid on
xlim([0 max(vehiclestate_plot_time)])
hold on

subplot(3,1,2)
plot(vehiclestate_plot_time,d_steer_angle.*5,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
plot(vehiclestate_plot_time,d_steer_angle.*5,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
plot(vehiclestate_plot_time,vehiclestate_steering_angle)
legend("auto","mannual","FontSize",10)
xlabel("time[s]","FontSize",20)
ylabel("steering angle speed[deg/s]","FontSize",20)
grid on
xlim([0 max(vehiclestate_plot_time)])
ylim([-5 5])
hold on

subplot(3,1,3)
plot(vehiclestate_plot_time,d_steer_cmd,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
plot(vehiclestate_plot_time,d_steer_cmd,'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
legend("auto","mannual","FontSize",10)
xlabel("time[s]","FontSize",20)
ylabel("steering cmd diff[deg/s]","FontSize",20)
grid on
xlim([0 max(vehiclestate_plot_time)])
ylim([-5 5])
set(gcf,'Position',[0 0 2600 1800]);
save_file_name = "vehiclestate_steer_diff"+ timestr + ".png";
saveas(gcf,save_file_name)
end
%%
plot_time_valid_index = find(vehiclestate_plot_time>=0);
positive_time_list = find(vehiclestate_plot_time>0);
if enable_plot_steer_cmd_feedback==1
vehiclestate_plot_time_diff = diff(vehiclestate_plot_time(positive_time_list));
vehiclestate_steering_angle_diff = diff(vehiclestate_steering_angle(positive_time_list));
vehiclestate_steering_cmd_diff = diff(vehiclestate_steering_cmd(positive_time_list));
vehiclestate_steering_angle_rate = [0;vehiclestate_steering_angle_diff ./ vehiclestate_plot_time_diff];
vehiclestate_steering_cmd_rate = [0;vehiclestate_steering_cmd_diff ./ vehiclestate_plot_time_diff];
figure(19)
subplot(2,1,1)
plot(vehiclestate_plot_time(positive_time_list),vehiclestate_steering_angle_rate);hold on
plot(vehiclestate_plot_time(positive_time_list),vehiclestate_steering_cmd_rate);hold on
legend("chassis steer rate","cmd steer rate","FontSize",20)
hold on 
grid on 
title("steering angle rate","Fontsize",20)
subplot(2,1,2)
plot(vehiclestate_plot_time(plot_time_valid_index),vehiclestate_steering_angle(plot_time_valid_index),'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
plot(vehiclestate_plot_time(plot_time_valid_index),vehiclestate_steering_angle(plot_time_valid_index),'.-',"MarkerIndices",vehiclestate_notdriving_indexlist,"MarkerEdgeColor","b");hold on
plot(vehiclestate_plot_time(plot_time_valid_index),vehiclestate_steering_cmd(plot_time_valid_index),'--',"MarkerEdgeColor","r");hold on
legend("auto","mannual","cmd","FontSize",20)
xlabel("time[s]","FontSize",20)
ylabel("steering angle[deg]","FontSize",20)
grid on
xlim([0 max(vehiclestate_plot_time)])
hold on 
set(gcf,'Position',[0 0 2600 1800]);
end
%%
check_speed_error_windowsize = 20;
localization2chassis_speed_error = zeros(length(vehiclestate_localization_velocity),1);
for i = 1:length(vehiclestate_localization_velocity)
    localization2chassis_speed_error(i) = vehiclestate_localization_velocity(i) - vehiclestate_linear_velocity(i);
end
mean_localization2chassis_accumulated_speed_error = zeros(length(vehiclestate_localization_velocity),1);
for i = check_speed_error_windowsize:length(vehiclestate_localization_velocity)
    localization2chassis_accumulated_speed_error(i) = sum(abs(localization2chassis_speed_error(i-check_speed_error_windowsize+1:i)));
    mean_localization2chassis_accumulated_speed_error(i) = mean(abs(localization2chassis_speed_error(i-check_speed_error_windowsize+1:i)));
end
localization2chassis_speed_error_list = [];
mean_speed = 2;
for i = check_speed_error_windowsize:length(vehiclestate_localization_velocity)
    mean_speed = mean(vehiclestate_linear_velocity(i-check_speed_error_windowsize+1:i));
    if abs(vehiclestate_linear_velocity(i))<0.1
       continue
    end
    if mean_localization2chassis_accumulated_speed_error(i)>mean_speed*0.15 ...
            || abs(localization2chassis_speed_error(i))>0.5
        localization2chassis_speed_error_list = [localization2chassis_speed_error_list;i];
    end

end
localization_calculated_velocity = vehiclestate_interpoint_distance./[0;diff(vehiclestate_timestamp)];
if enable_plot_speed_localization_chassis == 1
vehiclestate_chassis_check_code = vehiclestate(start_index:end,chassis_check_code_index);
chassis2localization_velocity_abnormal_list = [];
for i = 1:length(vehiclestate_localization_velocity)
    current_chassis_check_code_vector = f_dec2bin_vector(vehiclestate_chassis_check_code(i));
    if current_chassis_check_code_vector(8) == 1
        chassis2localization_velocity_abnormal_list = [chassis2localization_velocity_abnormal_list;i];
    end
end
figure(20)
subplot(3,1,1)
plot(vehiclestate_plot_time,vehiclestate_localization_velocity);hold on 
plot(vehiclestate_plot_time,vehiclestate_linear_velocity);hold on
plot(vehiclestate_plot_time,vehiclestate_linear_velocity,"g*","MarkerIndices",localization2chassis_speed_error_list);hold on
% plot(vehiclestate_plot_time,vehiclestate_linear_velocity,"ko","MarkerIndices",chassis2localization_velocity_abnormal_list);hold on
legend("localization v","chassis v","abnormal speed","Fontsize",20)
xlabel("time[s]","FontSize",20)
ylabel("speed[m/s]","FontSize",20)
title("localization speed vs chassis speed","FontSize",20)
grid on 
subplot(3,1,2)
plot(vehiclestate_plot_time,localization2chassis_speed_error);hold on
xlabel("time[s]","FontSize",20)
ylabel("speed error[m/s]","FontSize",20)
subplot(3,1,3)
grid on
plot(vehiclestate_plot_time,localization2chassis_accumulated_speed_error);hold on
xlabel("time[s]","FontSize",20)
ylabel("accumulated speed error[m/s]","FontSize",20)
grid on 
set(gcf,'Position',[0 0 2600 1800]);
end
%%
if enable_plot_cost_time ==1
figure(21)
subplot(2,1,1)
plot(vehiclestate_plot_time,vehiclestate_control_iteration_cost_ms./vehiclestate_control_iteration_cost_ms,'r-');hold on
plot(vehiclestate_plot_time,vehiclestate_pre_process_cost_ms./vehiclestate_control_iteration_cost_ms,'b-');hold on
plot(vehiclestate_plot_time,vehiclestate_produce_cmd_cost_ms./vehiclestate_control_iteration_cost_ms,'g-');hold on
plot(vehiclestate_plot_time,vehiclestate_post_process_cost_ms./vehiclestate_control_iteration_cost_ms,'y-')
legend('control iteration cost ms','pre','produce','post')
xlim([0 max(vehiclestate_plot_time)])
ylim([0 1])
subplot(2,1,2)
plot(vehiclestate_plot_time,vehiclestate_control_iteration_cost_ms,'r-');hold on
plot(vehiclestate_plot_time,vehiclestate_pre_process_cost_ms,'b-');hold on
plot(vehiclestate_plot_time,vehiclestate_produce_cmd_cost_ms,'g-');hold on
plot(vehiclestate_plot_time,vehiclestate_post_process_cost_ms,'y-')
legend('control iteration cost ms','pre','produce','post')
xlim([0 max(vehiclestate_plot_time)])
end
%%
if enable_plot_accumulated_error == 1
figure(22)
plot(vehiclestate_plot_time,vehiclestate_accumulated_error);hold on 
xlabel("time[s]","FontSize",20)
ylabel("accumulated error[m]","FontSize",20)
grid on 
end

%%

%%
figure(23)
subplot(2,1,1)
plot(vehiclestate_plot_time,vehiclestate_driving_mode,'Marker','o','Markersize',1,'Color','b'); hold on 
plot(vehiclestate_plot_time,vehiclestate_gear,'Marker','o','Markersize',1,'Color','r'); hold on 
xlabel("time[s]","FontSize",20)
ylabel("driving mode","FontSize",20)
legend("driving mode","gear","FontSize",20)
xlim([0 max(vehiclestate_plot_time)])
subplot(2,1,2)
plot(vehiclestate_plot_time,vehiclestate_standstill*1);hold on
plot(vehiclestate_plot_time,vehiclestate_require_standstill_steering*5);hold on
xlabel("time[s]","FontSize",20)
grid on 
legend("standstill","require standstill steering","FontSize",20)

cd ..