%% Localization csv after smooth:lateral_tuning_data.csv
% or using havp csv
%%
clear;
close all
csv_file_name = 'havp_learned_recording_processed_2';
glb_data = readmatrix(csv_file_name);
% x,y,z,heading,kappa,dkappa, s, left_bound, right_bound
%%
ts =0.1;
phi_index = 4;
curvature_index = 5;
if   csv_file_name(1:4) == 'havp'
    phi_index = 3;
    curvature_index = 4;
end
global_length = size(glb_data,1);
global_pose_x = glb_data(:,1);
global_pose_y = glb_data(:,2);
global_pose_phi = glb_data(:,phi_index);
global_curvature = glb_data(:,curvature_index);

global_distance = zeros(global_length,1);
global_lat_error = zeros(global_length,1);
lat_index = [];
for i =1:global_length-1
    current_pos = [global_pose_x(i),global_pose_y(i)];
    next_pos = [global_pose_x(i+1),global_pose_y(i+1)];
    global_distance(i,1)= norm(next_pos-current_pos);
    dx = global_pose_x(i+1) - global_pose_x(i);
    dy = global_pose_y(i+1) - global_pose_y(i);
    current_heading = global_pose_phi(i);
    if current_heading > -pi/2 && current_heading < pi/2
        alpha = abs(current_heading);
    else
        alpha = pi - abs(current_heading);
    end
    lat_error = cos(alpha) * dy - sin(alpha) * dx;
    global_lat_error(i) = lat_error;
    if abs(lat_error)>0.2
        lat_index = [lat_index;i];
    end
end
glb_data = [glb_data,global_distance];
index=[];
% for i =1:global_length-1
%     expect_x = global_pose_x(i) + cos(global_pose_phi(i)) * global_speed(i) *ts;
%     expect_y = global_pose_y(i) + sin(global_pose_phi(i)) * global_speed(i) *ts;
%     permit_distance_error =max(0.25,global_speed(i) *ts);
%     
%     next_pos = [global_pose_x(i+1),global_pose_y(i+1)];
%     expect_pos = [expect_x,expect_y];
%     distance_error = norm(next_pos - expect_pos);
%     if distance_error > permit_distance_error
%         index = [index;i];
%     end
% end

global_pose_phi_deg = zeros(global_length,1);
for i = 1:global_length
    global_pose_phi_deg(i) = global_pose_phi(i)/pi *180.0;
end
global_pose_phi_deg_delta = zeros(global_length,1);
for i = 2:global_length
    global_pose_phi_deg_delta(i) = global_pose_phi_deg(i) - global_pose_phi_deg(i-1);
end
heading_count = 0;
is_last_heading_normal = 1;
for i = 2:global_length
    if is_last_heading_normal==1 && abs(global_pose_phi_deg_delta(i)) > 1 && abs(global_pose_phi_deg_delta(i)) < 179
        heading_count = heading_count +1;
        index = [index;i];
        is_last_heading_normal = 0 ;
    elseif abs(global_pose_phi_deg_delta(i)) < 0.3
        is_last_heading_normal = 1 ;
    end
end
fprintf("heading_count  %d\n",heading_count);
heading_count_ratio = heading_count/global_length;
fprintf("heading_count_ratio  %.4f\n",heading_count_ratio);
%%
distance_count = 0;
for i = 1:global_length
    if i<10
        average_distance = mean(global_distance(1:20));
    elseif i>global_length-10
        average_distance = mean(global_distance(end-19:end));
    else 
        average_distance = mean(global_distance(i-9:i+10));
    end
    if abs(global_distance(i)-average_distance) > 0.05
        distance_count = distance_count+1;
    end
end
fprintf("distance_count  %d\n",distance_count);
distance_count_ratio = distance_count/global_length;
fprintf("distance_count_ratio  %.4f\n",distance_count_ratio);
%%
foldername = "pngfile_"+datestr(datetime);
mkdir(foldername);
cd(foldername)
%%
figure(1)
subplot(2,1,1)
% plot(global_pose_phi_deg,'o','MarkerSize',3);hold on
plot(global_pose_phi_deg);hold on
if size(index,1)>0
    for i = 1 :size(index,1)
        text(index(i),global_pose_phi_deg(index(i)),["X"],'color','b','FontSize',10);hold on
    end
end
ylabel('Phi[deg]',"FontSize",20)
title('heading',"FontSize",25)
grid on 
subplot(2,1,2)
plot(global_pose_phi_deg_delta,'o','MarkerSize',3);hold on
plot(global_pose_phi_deg_delta);
if size(index,1)>0
    for i = 1 :size(index,1)
        text(index(i),global_pose_phi_deg_delta(index(i)),["X"],'color','b','FontSize',10);hold on
    end
end
ylim([-5 5])
ylabel('delta Phi[deg]',"FontSize",20)
grid on
title('delta heading',"FontSize",25)
set(gcf,'Position',[0 0 2600 1800]);
save_file_name = "phi_deg.png";
saveas(gcf,save_file_name)

%%
figure(2)
% plot(global_pose_x,global_pose_y,'o','MarkerSize',3);hold on
plot(global_pose_x,global_pose_y,'*','MarkerSize',3,'Color',"k");hold on
xlabel("x[m]","FontSize",20)
ylabel("y[m]","FontSize",20)
grid on
set(gcf,'Position',[0 0 2600 1800]);
title('localization',"FontSize",25)
axis equal

figure(3)
plot(global_distance);hold on
plot(global_distance,'o','MarkerSize',3);hold on
axis([1 global_length 0 1]);
xlabel("index","FontSize",20)
ylabel("distance[m]","FontSize",20)
title('interpoint distance',"FontSize",25)
ylim([0 0.5])

%%
figure(4)
plot(global_curvature);hold on
grid on
axis([1 global_length 0 0.2]);
xlabel("index","FontSize",20)
ylabel("curvature","FontSize",20)
title('curvature',"FontSize",25)
set(gcf,'Position',[0 0 2600 1800]);
save_file_name = "curvature.png";
saveas(gcf,save_file_name)

figure(5)
subplot(2,1,1)
plot(global_pose_x,global_pose_y,'o','MarkerSize',3);hold on
if size(lat_index,1)>0
    for i = 1 :size(lat_index,1)
        text(global_pose_x(lat_index(i)),global_pose_y(lat_index(i)),["X"],'color','b','FontSize',10);hold on
    end
end
grid on
axis equal
xlabel("x","FontSize",20)
ylabel("y","FontSize",20)
title('position',"FontSize",25)

subplot(2,1,2)
plot(global_lat_error);hold on
if size(lat_index,1)>0
    for i = 1 :size(lat_index,1)
        text(lat_index(i),global_lat_error(lat_index(i)),["X"],'color','b','FontSize',10);hold on
    end
end
grid on 
axis([1 global_length -0.3 0.3]);
xlabel("index","FontSize",20)
ylabel("lateral error","FontSize",20)
title('lateral error',"FontSize",25)
set(gcf,'Position',[0 0 2600 1800]);
save_file_name = "lateral_error.png";
saveas(gcf,save_file_name)
%%
cd ..