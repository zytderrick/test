%% plot trajectory info
if enable_plot_traj_xy ==1
    figure(1)
    for i = trajectory_plot_range
        current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_x(plot_start_index:plot_end_index),...
            trajectory_y(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',7,'Marker',marker_list(mod(i,marker_size)+1));hold on
    end
    plot(vehiclestate_x,vehiclestate_y,'.',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r","MarkerSize",5);hold on
    plot(vehiclestate_x,vehiclestate_y,'.',"MarkerIndices",soft_brake_indexlist,"Marker","o","MarkerEdgeColor","g","MarkerSize",20);hold on
    xlabel("x[m]","FontSize",20)
    ylabel("y[m]","FontSize",20)
%     axis equal 
    grid on
    title("planning","FontSize",25)
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "traj_xy"+ timestr + ".png";
%     saveas(gcf,save_file_name)
end

if enable_plot_traj_theta ==1
    figure(2)
    for i = trajectory_plot_range
        current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_plot_time(plot_start_index:plot_end_index) + delta_time_traj_vehicle,...
            trajectory_theta_deg(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',5,'Marker',marker_list(mod(i,marker_size)+1));hold on  
    end
    xlabel("time[s]","FontSize",20)
    ylabel("theta[deg]","FontSize",20)
    ylim([-180 180])
    grid on
    title("planning","FontSize",25)
    xlim([trajectory_plot_time(1) trajectory_plot_time(end)+ + delta_time_traj_vehicle])
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "traj_heading"+ timestr + ".png";
%     saveas(gcf,save_file_name)
end

if enable_plot_traj_v ==1
    figure(3)
    subplot(2,1,1)
    for i = trajectory_plot_range
        current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_plot_time(plot_start_index:plot_end_index)  + delta_time_traj_vehicle,...
            trajectory_gear(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',3,'Marker',marker_list(mod(i,marker_size)+1))  
        hold on
        grid on
        ylabel("gear","FontSize",20)
    end
    subplot(2,1,2)
    for i = trajectory_plot_range
        current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_plot_time(plot_start_index:plot_end_index)  + delta_time_traj_vehicle,...
            trajectory_v(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',3,'Marker',marker_list(mod(i,marker_size)+1))  
        hold on
    end
%     plot(vehiclestate_plot_time,vehiclestate_linear_velocity,'.-',"MarkerIndices",vehiclestate_driving_indexlist,"MarkerEdgeColor","r");hold on
    plot(vehiclestate_plot_time,vehiclestate_localization_velocity);hold on 
    xlabel("time[s]","FontSize",20)
    ylabel("v[m/s]","FontSize",20)
    grid on
    title("planning","FontSize",25)
    xlim([trajectory_plot_time(1) trajectory_plot_time(end)])
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "traj_v"+ timestr + ".png";
%     saveas(gcf,save_file_name)
end

if enable_plot_traj_a ==1
    figure(4)
    for i = trajectory_plot_range
        current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_plot_time(plot_start_index:plot_end_index) + delta_time_traj_vehicle,...
            trajectory_a(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',3,'Marker',marker_list(mod(i,marker_size)+1))  
        hold on
    end
    xlabel("time[s]","FontSize",20)
    ylabel("a[m/s^2]","FontSize",20)
    grid on
    title("planning","FontSize",25)
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "traj_a"+ timestr + ".png";
%     saveas(gcf,save_file_name)
end
%% trajectory s
if enable_plot_traj_s ==1
    figure(5)
    subplot(2,1,1)
    for i = trajectory_plot_range
        current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_plot_time(plot_start_index:plot_end_index)  + delta_time_traj_vehicle,...
            trajectory_s(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',10,'Marker',marker_list(mod(i,marker_size)+1));hold on
    end
    xlabel("time[s]","FontSize",20)
    ylabel("s[m]","FontSize",20)
    grid on
    title("planning","FontSize",25)

    subplot(2,1,2)
    for i = trajectory_plot_range
        current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_plot_time(plot_start_index:plot_end_index)  + delta_time_traj_vehicle,...
            trajectory_accumulated_distatnce(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',10,'Marker',marker_list(mod(i,marker_size)+1));hold on
    end
    ylabel("accumulated distance[m]","FontSize",20)
    xlabel("time[s]","FontSize",20)
    grid on
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "traj_s"+ timestr + ".png";
%     saveas(gcf,save_file_name)
end
%% plot kappa
if enable_plot_traj_kappa ==1
    figure(6)
    for i = trajectory_plot_range
        current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_plot_time(plot_start_index:plot_end_index)  + delta_time_traj_vehicle,...
            trajectory_kappa(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',3,'Marker',marker_list(mod(i,marker_size)+1));hold on
    end
    xlabel("time[s]","FontSize",20)
    ylabel("kappa","FontSize",20)
    grid on
    title("planning","FontSize",25)
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "traj_kappa"+ timestr + ".png";
%     saveas(gcf,save_file_name)
end
%% trajectory distance
if enable_plot_traj_distance ==1
    figure(7)
    for i = trajectory_plot_range
        current_trajectory_index_list = find(trajectory_index==i);
        plot_start_index = current_trajectory_index_list(1);
        plot_end_index = min(current_trajectory_index_list(1)+number_plot_point_each_instance,current_trajectory_index_list(end));
        plot(trajectory_plot_time(plot_start_index:plot_end_index)  + delta_time_traj_vehicle,...
            trajectory_interpoint_distatnce(plot_start_index:plot_end_index),color_list(mod(i,color_size)+1),'MarkerSize',10,'Marker',marker_list(mod(i,marker_size)+1));hold on
    end
    xlabel("time[s]","FontSize",20)
    ylabel("interpoint distance[m]","FontSize",20)
    grid on
    title("planning","FontSize",25)
    set(gcf,'Position',[0 0 2600 1800]);
    save_file_name = "traj_distance"+ timestr + ".png";
%     saveas(gcf,save_file_name)
end