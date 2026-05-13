%% stop evaluation using parking spot
load('saved_slot_pos.mat');
load("common.mat")
%%
driving_mode_vector = find(vehiclestate_driving_mode==1);
start_driving_index = 1;
for i = 1: length(vehiclestate_driving_mode)-1
    if vehiclestate_driving_mode(i) == 1 && vehiclestate_linear_velocity(i) > 0.2
        start_driving_index = i;
        break;
    end
end
stop_point_index = driving_mode_vector(end);
vehicle_stop_xytheta = [];
for i=start_driving_index+1:stop_point_index-1
    if vehiclestate_driving_mode(i) ~= 1 
        continue
    end 
    if vehiclestate_standstill(i) == 1 && vehiclestate_gear(i)~=vehiclestate_gear(i+1)
        vehicle_stop_xytheta = [vehicle_stop_xytheta;vehiclestate_x(i),vehiclestate_y(i),vehiclestate_heading(i)];
    end
end
if isempty(vehicle_stop_xytheta)
    fprintf("stop_evaluation: no stop points! ---return---\n");
    return
end
final_stop_point = vehicle_stop_xytheta(end,:);
%% trajectory stop point
% try to separate two situation: 1.not to stop 2.had stopped
traj_stop_xytheta = [];
traj_stop_index_list = [];
is_going_to_stop = 0;
for i = 1:max(trajectory_index)
    single_trajectory_list = find(trajectory_index==i);
    if length(single_trajectory_list)>20
        single_trajectory_list = single_trajectory_list(1:20);
    end
    single_trajectory_x = trajectory_x(single_trajectory_list);
    single_trajectory_y = trajectory_y(single_trajectory_list);
    single_trajectory_s = trajectory_s(single_trajectory_list);
    single_trajectory_v = trajectory_v(single_trajectory_list);
    single_trajectory_gear = trajectory_gear(single_trajectory_list);

    last_is_going_to_stop = 0;
    for j = 2:length(single_trajectory_list)-1
        if abs(single_trajectory_s(j)-single_trajectory_s(j+1))<1e-5 && abs(single_trajectory_s(j-1)-single_trajectory_s(j))<1e-5 && single_trajectory_s(j)~=0
            is_going_to_stop = 1;
            current_index = single_trajectory_list(j);
            tmp_traj_stop_xytheta = [trajectory_x(current_index),trajectory_y(current_index),trajectory_theta(current_index)];
            last_is_going_to_stop = is_going_to_stop;
            break
        end
        if j==length(single_trajectory_list)-1
            is_going_to_stop = 0;
            if last_is_going_to_stop==1
                traj_stop_xytheta = [traj_stop_xytheta;tmp_traj_stop_xytheta];
                traj_stop_index_list = [traj_stop_index_list;current_index];
            end
        end
        last_is_going_to_stop = is_going_to_stop;
    end
end

if size(vehicle_stop_xytheta,1) ~= size(traj_stop_xytheta,1) || isempty(vehicle_stop_xytheta)
    fprintf("The number of stop points for vehicle differs from trajectory\n");
else
    veh2tra_stop_lat_error_list = [];
    veh2tra_stop_sta_error_list = [];
    veh2tra_stop_head_error_list = [];
    name_list = [];
    for k = 1:size(vehicle_stop_xytheta,1)
        veh2tra_stop_gear = trajectory_gear(traj_stop_index_list(k));
        if veh2tra_stop_gear == 2
            gear_coeff = -1;
        else
            gear_coeff = 1; 
        end
        [lateral_error,station_error] = f_compute_lateral_station_error(vehicle_stop_xytheta(k,1),vehicle_stop_xytheta(k,2),...
            traj_stop_xytheta(k,1),traj_stop_xytheta(k,2),traj_stop_xytheta(k,3));
        heading_error_rad = f_normalize_angle(vehicle_stop_xytheta(k,3) - traj_stop_xytheta(k,3));
        veh2tra_stop_lat_error_list = [veh2tra_stop_lat_error_list;lateral_error];
        veh2tra_stop_sta_error_list = [veh2tra_stop_sta_error_list;station_error*gear_coeff];
        veh2tra_stop_head_error_list = [veh2tra_stop_head_error_list;heading_error_rad*180/pi]; 
        name_list = [name_list;"stop point"+string(k)];
        if enable_plot_traj_xy == 1
        figure(1)
        plot(vehicle_stop_xytheta(k,1),vehicle_stop_xytheta(k,2),"Marker","v","MarkerEdgeColor","red","MarkerSize",15,"LineWidth",3)
        plot(traj_stop_xytheta(k,1),traj_stop_xytheta(k,2),"Marker","<","MarkerEdgeColor","blue","MarkerSize",15,"LineWidth",3)
        end
        if enable_plot_traj_s ==1
        figure(5)
        subplot(2,1,1)
        plot(trajectory_plot_time(traj_stop_index_list(k))+delta_time_traj_vehicle,...
            trajectory_s(traj_stop_index_list(k)),"Marker","<","MarkerEdgeColor","blue","MarkerSize",15,"LineWidth",3);
        end
    end
    veh2tra_stop_table = table(name_list,veh2tra_stop_head_error_list,veh2tra_stop_lat_error_list,veh2tra_stop_sta_error_list,'VariableNames',{'index','hdg error(deg)','lat error(m)','stat error(m)'});
    disp(veh2tra_stop_table);
end
return
%% ref_x ref_y ref_heading
ref_parking_spot_list = [];
ref_stop2base_center = 0.93+0.6;
for i = 1:size(slot_list,1)
    spot_position = slot_list(i,:);
    base_center_x = (spot_position(5)+spot_position(7))/2;
    base_center_y = (spot_position(6)+spot_position(8))/2;
    ref_heading_rad = atan2((spot_position(2)-spot_position(8)),(spot_position(1)-spot_position(7)));
    ref_stop_x = base_center_x + cos(ref_heading_rad) * ref_stop2base_center;
    ref_stop_y = base_center_y + sin(ref_heading_rad) * ref_stop2base_center;
    ref_parking_spot_list = [ref_parking_spot_list;ref_stop_x,ref_stop_y,ref_heading_rad];
end
min_stop2parkingspot_distance = 100;
matched_ref_index = 1;
for i = 1:length(ref_parking_spot_list)
    distance = norm([ref_parking_spot_list(i,1)-final_stop_point(1,1),ref_parking_spot_list(i,2)-final_stop_point(1,2)]);
    if distance < min_stop2parkingspot_distance
       min_stop2parkingspot_distance = distance;
       matched_ref_index = i;
    end
end
matched_ref_x = ref_parking_spot_list(matched_ref_index,1);
matched_ref_y = ref_parking_spot_list(matched_ref_index,2);
matched_ref_heading_rad = ref_parking_spot_list(matched_ref_index,3);
heaiding_error_rad = final_stop_point(1,3) - matched_ref_heading_rad;
heaiding_error_rad = f_normalize_angle(heaiding_error_rad);
gear_coefficient = 1;
for j = stop_point_index:-1:1
if vehiclestate_gear(j)==2
    gear_coefficient = -1;
    break
elseif vehiclestate_gear(j)==1
    gear_coefficient = 1;
    break
else
    continue
end
end
heaiding_error_deg = heaiding_error_rad *180/pi;
[stop_lateral_error,stop_station_error] = f_compute_lateral_station_error(final_stop_point(1,1),final_stop_point(1,2),matched_ref_x,matched_ref_y,matched_ref_heading_rad);
stop_evaluation_list = ["heading error(deg)";"lateral error(m)";"station error(m)"];
stop_evaluation_value = [heaiding_error_deg;stop_lateral_error;stop_station_error*gear_coefficient];
stop_evaluation = table(stop_evaluation_list,stop_evaluation_value,'VariableNames',{'stop evaluation','stop value'});
disp(stop_evaluation)

%%
ref_parking_slot_position = slot_list(matched_ref_index,:);
ref_parking_spot_rect_x = [ref_parking_slot_position(1),ref_parking_slot_position(3),ref_parking_slot_position(5),...
                            ref_parking_slot_position(7),ref_parking_slot_position(1)];
ref_parking_spot_rect_y = [ref_parking_slot_position(2),ref_parking_slot_position(4),ref_parking_slot_position(6),...
                            ref_parking_slot_position(8),ref_parking_slot_position(2)];
if enable_plot_localization ==1
figure(8)
plot(final_stop_point(1,1),final_stop_point(1,2),"Marker","v","MarkerSize",15,"LineWidth",3);hold on
plot(matched_ref_x,matched_ref_y,"Marker",">","MarkerSize",15,"LineWidth",3);hold on
line(ref_parking_spot_rect_x,ref_parking_spot_rect_y,"color",'k','Linewidth',3);
legend("auto","mannual","gear reverse","soft brake","start point","localization jump","stop point","ref stop point","parking slot","FontSize",20)
axis equal
end
if enable_plot_traj_xy==1
figure(1)
plot(final_stop_point(1,2),final_stop_point(1,2),"Marker","v","MarkerSize",15,"LineWidth",3);hold on
plot(matched_ref_x,matched_ref_y,"Marker",">","MarkerSize",15,"LineWidth",3);hold on
line(ref_parking_spot_rect_x,ref_parking_spot_rect_y,"color",'k','Linewidth',3);
axis equal
end