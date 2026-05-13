%% analysis steering angle
function f_analysis_steer(vehiclestate,trajectory)
%%
load("common.mat");
%%
steer_angel_limit = [0;1;3;5];
vehiclestate_driving_mode = vehiclestate(:,driving_mode_index);
driving_mode_index_list = find(vehiclestate_driving_mode==1);
%%
timestamp = vehiclestate(:,timestamp_sec_index);
x = vehiclestate(:,raw_x_index);
y = vehiclestate(:,raw_y_index);
steering_angle = vehiclestate(:,steering_angle_index);
data_length = length(steering_angle);

trajectory_index = trajectory(1:end-1,trajectory_index_index);
trajectory_x = trajectory(1:end-1,trajectory_x_index);
trajectory_y = trajectory(1:end-1,trajectory_y_index);
trajectory_theta = trajectory(1:end-1,trajectory_theta_index);
trajectory_kappa= trajectory(1:end-1,trajectory_kappa_index);
trajectory_dkappa= trajectory(1:end-1,trajectory_dkappa_index);
trajectory_relativetime = trajectory(1:end-1,trajectory_relativetime_index);
trajectory_headertime = trajectory(1:end-1,trajectory_headertime_index);

trajectory_time = trajectory_headertime(:) - trajectory_headertime(1);
trajectory_time = trajectory_time + trajectory_relativetime;


%% find kappa for every point
% 1.find matched point with kappa
% 2.calculate filtered kappa for sequantial point
% 3.reduce intrustable kappa
matched_curvature = zeros(data_length,1);
matched_curvature_rate = zeros(data_length,1);
matched_SWA_deg = zeros(data_length,1);
shrink_curvature = zeros(data_length,1);
% match
% fprintf("match kappa process");
% tic
for i = 1: data_length
    current_x = x(i);
    current_y = y(i);
    current_time = timestamp(i) - timestamp(1);
    closest_trajectory_index = f_find_closest_point(current_x,current_y,current_time,trajectory_x,trajectory_y,trajectory_time);
    matched_curvature(i) = trajectory_kappa(closest_trajectory_index);
    matched_curvature_rate(i) = trajectory_dkappa(closest_trajectory_index);
    matched_SWA_deg(i) = f_kappa2SWAdeg(matched_curvature(i));
    matched_point = [trajectory_x(closest_trajectory_index),trajectory_y(closest_trajectory_index)];
    current_point = [current_x,current_y];
%     if norm(matched_point-current_point)>0.3
%         matched_curvature(i) =  matched_curvature(i-1);
%         matched_curvature_rate(i) = matched_curvature_rate(i-1);
%     end
end
% toc
% mean
kappa_window_size = 20;
kappa_filter_b = (1/kappa_window_size)*ones(1,kappa_window_size);
kappa_filter_a = 1;
mean_curvature = filter(kappa_filter_b,kappa_filter_a,matched_curvature);

dkappa_window_size = 20;
dkappa_filter_b = (1/dkappa_window_size)*ones(1,dkappa_window_size);
dkappa_filter_a = 1;
mean_curvature_rate = filter(dkappa_filter_b,dkappa_filter_a,matched_curvature_rate);

for i = 1: data_length
    matched_meankappa2SWA_deg(i) = f_kappa2SWAdeg(mean_curvature(i));
end
matched_meankappa2SWA_deg = matched_meankappa2SWA_deg';
% shrink straight kappa 
judge_turn_former_length = 200;
judge_turn_latter_length = 200;
for i = 1: data_length
    judge_turn_former_index = max(1,i-judge_turn_former_length+1);
    judge_turn_latter_index = min(data_length,i+judge_turn_latter_length);
    if max(abs(mean_curvature(judge_turn_former_index:judge_turn_latter_index))) < 0.05
        shrink_curvature(i) = mean_curvature(i)/3;
    else 
        shrink_curvature(i) = mean_curvature(i);
    end
end
%% get turn list and straight is left
% 1.tell turn using clear curvature
% 2.extend continuous turning
Index = 1:data_length;
Index = Index';
road_list = zeros(data_length,1);
for i = 1:data_length
    if abs(shrink_curvature(i)) > 0.02
        road_list(i) = 1;
        for j = i-1:-1:1
            if(abs(shrink_curvature(j))<1e-3)
                break
            end
            road_list(j) = 1;
        end
        for j = i+1:data_length-1
            if(abs(shrink_curvature(j))<1e-3)
                break
            end
            road_list(j) = 1;
        end
    end
%     if abs(mean_curvature_rate(i)) > 0.05
%         road_list(i) = 2;
%     end
    if i>30 && i<data_length-30
        if max(abs(shrink_curvature(i-30+1:i+30))) >0.05
            road_list(i) = 1;
        end
    end
    if i>120 && i<data_length-120
        if max(abs(shrink_curvature(i-120+1:i+120))) >0.1
            road_list(i) = 1;
        end
    end
end

for i = 1:data_length-20
    if road_list(i) == 0 && road_list(i+20) == 1
        road_list(i) = 1;
    end
end

turn_list = Index(road_list==1);
straight_list = Index(road_list==0);
%% get steering angle reverse point
d_steering_angle = [0;diff(steering_angle)];
d_steering_angle_window_size = 20;
d_steering_angle_filter_b = (1/d_steering_angle_window_size)*ones(d_steering_angle_window_size,1);
d_steering_angle_filter_a = 1;
mean_d_steering_angle = filter(d_steering_angle_filter_b,d_steering_angle_filter_a,d_steering_angle);
%% straight invert
steer_invert_list_tmp = [];
i = 1;
while i < data_length 
    if abs(d_steering_angle(i)) < 0.2
        i = i+1;
        continue
    end
    current_d_steering_angle = d_steering_angle(i);
    for j = i+1 :data_length - 10
        if d_steering_angle(j)*current_d_steering_angle < 0
            steer_invert_list_tmp = [steer_invert_list_tmp;j];
            break
        end
    end
    i = j+1;
end
% screen abnormal steer invert
% abnormal diff steering angle is over x deg
straight_steer_invert_standard_diff = 8;
abnormal_steer_invert_list_tmp = [];
for i = 2:length(steer_invert_list_tmp)
    last_index = steer_invert_list_tmp(i-1);
    index = steer_invert_list_tmp(i);
    if abs(steering_angle(index) - steering_angle(last_index)) < straight_steer_invert_standard_diff ...
        || sum(abs(d_steering_angle(index-10:index+10))) < 2
        continue
    end
    abnormal_steer_invert_list_tmp = [abnormal_steer_invert_list_tmp;index];
end
tabulate_list_1 = tabulate([straight_list;abnormal_steer_invert_list_tmp]);
steer_invert_straight_list = find(tabulate_list_1(:,2)>1);
%% turn invert
steer_invert_list_tmp = [];
i = 1;
while i < data_length 
    if abs(d_steering_angle(i)) < 0.2
        i = i+1;
        continue
    end
    current_d_steering_angle = d_steering_angle(i);
    for j = i+1 :data_length - 10
        if d_steering_angle(j)*current_d_steering_angle < 0
            steer_invert_list_tmp = [steer_invert_list_tmp;j];
            break
        end
    end
    i = j+1;
end
% screen abnormal steer invert
% abnormal diff steering angle is over x deg
turn_steer_invert_standard_diff = 20;
abnormal_steer_invert_list_tmp = [];
for i = 2:length(steer_invert_list_tmp)
    last_index = steer_invert_list_tmp(i-1);
    index = steer_invert_list_tmp(i);
    if abs(steering_angle(index) - steering_angle(last_index)) < turn_steer_invert_standard_diff ...
            || sum(abs(d_steering_angle(index-10:index+10)))< 2
        continue
    end
    abnormal_steer_invert_list_tmp = [abnormal_steer_invert_list_tmp;index];
end
tabulate_list_2 = tabulate([turn_list;abnormal_steer_invert_list_tmp]);
steer_invert_turn_list = find(tabulate_list_2(:,2)>1);
%% get all abnormal steer invert index 
steer_invert_list = sort([steer_invert_straight_list;steer_invert_turn_list]);
%% kappa evaluation same as above
% kappa stright invert
d_kappa2SWA = [0;diff(matched_meankappa2SWA_deg)];
kappa_invert_list_tmp = [];
i = 1;
while i < data_length 
    if abs(d_kappa2SWA(i)) < 0.2
        i = i+1;
        continue
    end
    current_d_kappa2SWA = d_kappa2SWA(i);
    for j = i+1 :data_length - 10
        if d_kappa2SWA(j)*current_d_kappa2SWA < 0
            kappa_invert_list_tmp = [kappa_invert_list_tmp;j];
            break
        end
    end
    i = j+1;
end
% screen abnormal steer invert
% abnormal diff steering angle is over x deg
straight_kappa_invert_standard_diff = 12;
abnormal_kappa_invert_list_tmp = [];
for i = 2:length(kappa_invert_list_tmp)
    last_index = kappa_invert_list_tmp(i-1);
    index = kappa_invert_list_tmp(i);
    if abs(matched_meankappa2SWA_deg(index) - matched_meankappa2SWA_deg(last_index)) < straight_kappa_invert_standard_diff ...
        || sum(abs(d_kappa2SWA(index-10:index+10))) < 5
        continue
    end
    abnormal_kappa_invert_list_tmp = [abnormal_kappa_invert_list_tmp;index];
end
tabulate_list_3 = tabulate([straight_list;abnormal_kappa_invert_list_tmp]);
kappa_invert_straight_list = find(tabulate_list_3(:,2)>1);
%% kappa turn invert
kappa_invert_list_tmp = [];
i = 1;
while i < data_length 
    if abs(d_kappa2SWA(i)) < 0.2
        i = i+1;
        continue
    end
    current_d_kappa2SWA = d_kappa2SWA(i);
    for j = i+1 :data_length - 10
        if d_kappa2SWA(j)*current_d_kappa2SWA < 0
            kappa_invert_list_tmp = [kappa_invert_list_tmp;j];
            break
        end
    end
    i = j+1;
end
% screen abnormal steer invert
% abnormal diff steering angle is over x deg
turn_kappa_invert_standard_diff = 20;
abnormal_kappa_invert_list_tmp = [];
for i = 2:length(kappa_invert_list_tmp)
    last_index = kappa_invert_list_tmp(i-1);
    index = kappa_invert_list_tmp(i);
    if abs(matched_meankappa2SWA_deg(index) - matched_meankappa2SWA_deg(last_index)) < turn_kappa_invert_standard_diff ...
            || sum(abs(d_kappa2SWA(index-10:index+10))) < 5
        continue
    end
    abnormal_kappa_invert_list_tmp = [abnormal_kappa_invert_list_tmp;index];
end
tabulate_list_4 = tabulate([turn_list;abnormal_kappa_invert_list_tmp]);
kappa_invert_turn_list = find(tabulate_list_4(:,2)>1);
%%
kappa_invert_list = sort([kappa_invert_straight_list;kappa_invert_turn_list]);
%% make evaluation
straight_list_length = length(straight_list);
turn_list_length = length(turn_list);
straight_time_length = straight_list_length * control_ts;
turn_time_length = turn_list_length * control_ts;
steer_invert_straight_count = length(steer_invert_straight_list);
steer_invert_turn_count = length(steer_invert_turn_list);
evaluation_list = ["straight";"turn";"total"];
time_length_list = [straight_time_length;turn_time_length;straight_time_length+turn_time_length];
steer_invert_count_list = [steer_invert_straight_count;steer_invert_turn_count;steer_invert_straight_count+steer_invert_turn_count];
steer_invert_count_per_10_sec_list = steer_invert_count_list./time_length_list *10;
steer_evaluation_table = table(evaluation_list,time_length_list,steer_invert_count_list,steer_invert_count_per_10_sec_list,'VariableNames',{'steer_info','time length[s]','tp_count','tp_count per 10s'});
% disp(steer_evaluation_table);

kappa_invert_straight_count = length(kappa_invert_straight_list);
kappa_invert_turn_count = length(kappa_invert_turn_list);
kappa_evaluation_list = ["straight";"turn";"total"];
kappa_invert_count_list = [kappa_invert_straight_count;kappa_invert_turn_count;kappa_invert_straight_count+kappa_invert_turn_count];
kappa_invert_count_per_10_sec_list = kappa_invert_count_list./time_length_list *10;
kappa_evaluation_table = table(kappa_evaluation_list,time_length_list,kappa_invert_count_list,kappa_invert_count_per_10_sec_list,'VariableNames',{'kapp_info','time length[s]','tp_count','tp_count per 10s'});
% disp(kappa_evaluation_table);

coalitional_table = table(["steer";"kappa"],[steer_invert_count_list(end);kappa_invert_count_list(end)],[steer_invert_count_per_10_sec_list(end);kappa_invert_count_per_10_sec_list(end)],'VariableNames',{'info','tp_cnt','tp_cnt per 10s'});
disp(coalitional_table)
fprintf("tp_cnt: turning point count\n");
%%
figure(41)
subplot(2,1,1)
plot(x,y,"o","MarkerIndices",straight_list,"MarkerEdgeColor","r","DisplayName","straight driving","MarkerSize",3);hold on
plot(x,y,".","MarkerIndices",turn_list,"MarkerEdgeColor","g","DisplayName","turning");hold on
plot(x,y,"*","MarkerIndices",steer_invert_list,"MarkerEdgeColor","k","DisplayName","turning");hold on
xlabel("X[m]","FontSize",20)
ylabel("Y[m]","FontSize",20)
legend("straight","turn","steer invert","Fontsize",20)
grid on
axis equal

subplot(2,1,2)
plot(steering_angle,"o","MarkerIndices",straight_list,"MarkerEdgeColor","r","DisplayName","straight driving","MarkerSize",3);hold on
plot(steering_angle,".","MarkerIndices",turn_list,"MarkerEdgeColor","g","DisplayName","turning");hold on
plot(steering_angle,"*","MarkerIndices",steer_invert_list,"MarkerEdgeColor","k","DisplayName","turning");hold on
legend("straight","turn","steer invert","Fontsize",20)
grid on
xlabel("index","FontSize",20)
ylabel("steering angle[deg]","FontSize",20)
%%
figure(42)
subplot(3,1,1)
plot(matched_curvature,'r');hold on 
plot(mean_curvature,'g');hold on 
plot(shrink_curvature,'b');hold on 
grid on
ylabel("kappa","FontSize",20)
legend("matched","mean","shrink","FontSize",20)

subplot(3,1,2)
plot(matched_curvature_rate,'r');hold on
plot(mean_curvature_rate,'g');hold on 
plot(d_steering_angle,'b');hold on
plot(mean_d_steering_angle,'y');hold on
grid on 
ylabel("dkappa","FontSize",20)
legend("matched","mean","diff SWA","mean diff SWA","FontSize",20)

subplot(3,1,3)
plot(steering_angle,"o","MarkerIndices",straight_list,"MarkerEdgeColor","r","DisplayName","straight driving","MarkerSize",3);hold on
plot(steering_angle,".","MarkerIndices",turn_list,"MarkerEdgeColor","g","DisplayName","turning");hold on
plot(steering_angle,"*","MarkerIndices",steer_invert_list,"MarkerEdgeColor","k");hold on
plot(matched_meankappa2SWA_deg,"bo","MarkerSize",2);hold on
plot(matched_meankappa2SWA_deg,"o","MarkerIndices",kappa_invert_list,"MarkerEdgeColor","k","MarkerSize",7);hold on
grid on
ylabel("steering angle","FontSize",20)
legend("straight","turn","invert","kappa->SWA","kapa invert","Fontsize",20)
title("SWA vs kappa",FontSize=20)
end
%%    
