function [abnormal_index_list]=f_planning_evaluate(trajectory)
%%
load("common.mat");
%%
current2last_frame_index_diff = floor(control_ts * trajectorylog_control_period/planning_d_relavtive_time);
abnormal_index_list =[];
%% get data
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
%%
trajectory_plot_time = trajectory_headertime(:) - trajectory_headertime(1);
trajectory_plot_time = trajectory_plot_time(:) + trajectory_relativetime(:);
trajectory_kappa2SWA = zeros(trajectory_total_length,1);
%% speed
if enable_singleframe_v_a_kappa_check == 1
v_tp_cnt = zeros(max(trajectory_index),1);
integral_d_v = zeros(max(trajectory_index),1);
for i = 1 : max(trajectory_index)
    single_trajectory_list = find(trajectory_index==i);
    if length(single_trajectory_list)>100
        single_trajectory_list = single_trajectory_list(1:end-1);
    end
    single_trajectory_v = trajectory_v(single_trajectory_list);
    single_trajectory_time = trajectory_plot_time(single_trajectory_list);
    v_tp_cnt(i) = f_turing_point_count(single_trajectory_v,single_trajectory_time,101);
    integral_d_v(i) = sum(abs(diff(single_trajectory_v)));
%     if v_tp_cnt(i) >2
%         abnormal_index_list = [abnormal_index_list;i];
%     end
end
%% acc
a_tp_cnt = zeros(max(trajectory_index),1);
integral_d_a = zeros(max(trajectory_index),1);
a_overlimit_cnt = zeros(max(trajectory_index),1);
acc_scenario_cnt = 0;
dec_scenario_cnt = 0;
for i = 1 : max(trajectory_index)
    single_trajectory_list = find(trajectory_index==i);
    if length(single_trajectory_list)>100
        single_trajectory_list = single_trajectory_list(1:end-1);
    end
    single_trajectory_a = trajectory_a(single_trajectory_list);
    single_trajectory_time = trajectory_plot_time(single_trajectory_list);
    a_tp_cnt(i) = f_turing_point_count(single_trajectory_a,single_trajectory_time,105);
    integral_d_a(i) = sum(abs(diff(single_trajectory_a)));
    a_overlimit_cnt(i) = length(find(single_trajectory_a>a_ub))+length(find(single_trajectory_a<a_lb));
%     if a_tp_cnt(i) >2
%         abnormal_index_list = [abnormal_index_list;i];
%     end
    if ~isempty(find(single_trajectory_a>1))
        acc_scenario_cnt = acc_scenario_cnt + 1;
    end
    if ~isempty(find(single_trajectory_a<-2))
        dec_scenario_cnt = dec_scenario_cnt + 1;
    end
end
max_a_overlimit_cnt = max(a_overlimit_cnt);
TP99_a_overlimit_cnt = f_TP_ratio_value(a_overlimit_cnt,0.99);
TP95_a_overlimit_cnt = f_TP_ratio_value(a_overlimit_cnt,0.95);
TP90_a_overlimit_cnt = f_TP_ratio_value(a_overlimit_cnt,0.9);
times_a_overlimit_cnt = length(find(a_overlimit_cnt>0));
%% kappa
for i = 1:trajectory_total_length
    trajectory_kappa2SWA(i) = f_kappa2SWAdeg(trajectory_kappa(i));
end
kappa_tp_cnt = zeros(max(trajectory_index),1);
integral_d_kappa = zeros(max(trajectory_index),1);
integral_d_kappa2SWA = zeros(max(trajectory_index),1);
kappa_overlimit_cnt = zeros(max(trajectory_index),1);
dkappa_overlimit_cnt = zeros(max(trajectory_index),1);
for i = 1 : max(trajectory_index)
    single_trajectory_list = find(trajectory_index==i);
    if length(single_trajectory_list)>100
        single_trajectory_list = single_trajectory_list(1:end-1);
    end
    single_trajectory_kappa = trajectory_kappa(single_trajectory_list);
    single_trajectory_dkappa = trajectory_dkappa(single_trajectory_list);
    single_trajectory_kappa2SWA = trajectory_kappa2SWA(single_trajectory_list);
    single_trajectory_time = trajectory_plot_time(single_trajectory_list);
    kappa_tp_cnt(i) = f_turing_point_count(single_trajectory_kappa,single_trajectory_time,111);
    integral_d_kappa(i) = sum(abs(diff(single_trajectory_kappa)));
    integral_d_kappa2SWA(i) = sum(abs(diff(single_trajectory_kappa2SWA)));
    kappa_overlimit_cnt(i) = length(find(abs(single_trajectory_kappa)>kappa_limit));
    dkappa_overlimit_cnt(i) = length(find(abs(single_trajectory_dkappa)>dkappa_limit));
%     if kappa_tp_cnt(i) > 2
%         abnormal_index_list = [abnormal_index_list;i];
%     end
end
% kappa over limit evaluate
fprintf("\t \t \tsingle frame kappa evaluation\n");
max_kappa_overlimit_cnt = max(kappa_overlimit_cnt);
TP99_kappa_overlimit_cnt = f_TP_ratio_value(kappa_overlimit_cnt,0.99);
TP95_kappa_overlimit_cnt = f_TP_ratio_value(kappa_overlimit_cnt,0.95);
TP90_kappa_overlimit_cnt = f_TP_ratio_value(kappa_overlimit_cnt,0.9);
times_kappa_overlimit_cnt = length(find(kappa_overlimit_cnt>0));
max_dkappa_overlimit_cnt = max(dkappa_overlimit_cnt);
TP99_dkappa_overlimit_cnt = f_TP_ratio_value(dkappa_overlimit_cnt,0.99);
TP95_dkappa_overlimit_cnt = f_TP_ratio_value(dkappa_overlimit_cnt,0.95);
TP90_dkappa_overlimit_cnt = f_TP_ratio_value(dkappa_overlimit_cnt,0.9);
times_dkappa_overlimit_cnt = length(find(dkappa_overlimit_cnt>0));
max_overlimit_cnt = [max_a_overlimit_cnt;max_kappa_overlimit_cnt;max_dkappa_overlimit_cnt];
TP99_overlimit_cnt = [TP99_a_overlimit_cnt;TP99_kappa_overlimit_cnt;TP99_dkappa_overlimit_cnt];
TP95_overlimit_cnt = [TP95_a_overlimit_cnt;TP95_kappa_overlimit_cnt;TP95_dkappa_overlimit_cnt];
TP90_overlimit_cnt = [TP90_a_overlimit_cnt;TP90_kappa_overlimit_cnt;TP90_dkappa_overlimit_cnt];
times_overlimit_cnt = [times_a_overlimit_cnt;times_kappa_overlimit_cnt;times_dkappa_overlimit_cnt];
table0 = table(["a count";"kappa count";"dkappa count"],max_overlimit_cnt,TP99_overlimit_cnt,TP95_overlimit_cnt,...
    TP90_overlimit_cnt,times_overlimit_cnt,'VariableNames',{'overlimit','max','TP99','TP95','TP90','times'});
disp(table0);
%
fprintf("\t \t \tsingle frame analysis\n");
name_list = ["v(m/s)";"a(m/s^2)";"kappa";"kappa->SWA(deg)"];
sum_tp_cnt_list = [sum(v_tp_cnt);sum(a_tp_cnt);sum(kappa_tp_cnt);sum(kappa_tp_cnt)];
average_cnt_list = [sum(v_tp_cnt)/length(v_tp_cnt);sum(a_tp_cnt)/length(a_tp_cnt);sum(kappa_tp_cnt)/length(kappa_tp_cnt);sum(kappa_tp_cnt)/length(kappa_tp_cnt)];
max_tp_cnt_list = [max(v_tp_cnt);max(a_tp_cnt);max(kappa_tp_cnt);max(kappa_tp_cnt)];
max_integral_diff_list = [max(integral_d_v);max(integral_d_a);max(integral_d_kappa);max(integral_d_kappa2SWA)];
TP99_integral_diff_list = [f_TP_ratio_value(integral_d_v,0.99);f_TP_ratio_value(integral_d_a,0.99);f_TP_ratio_value(integral_d_kappa,0.99);f_TP_ratio_value(integral_d_kappa2SWA,0.99)];
table1 = table(name_list,sum_tp_cnt_list,average_cnt_list,max_tp_cnt_list,max_integral_diff_list,TP99_integral_diff_list,'VariableNames',{'info','sum_tp_cnt','ave_tp_cnt','max_tp_cnt','max_intd','TP99_intd'});
disp(table1);
fprintf("   tp(拐点):turning point\n   ave:average\n   intd（差值积分）:integral diff\n");
end
%% 帧间一致
if enable_interframe_diff_check == 1
% 帧间 diff的sum，max，point cnt
% max(sum), TP(max), TP(sum/cnt)
xy_diff = zeros(max(trajectory_index),3);
theta_diff = zeros(max(trajectory_index),3);
v_diff = zeros(max(trajectory_index),3);
a_diff = zeros(max(trajectory_index),3);
kappa_diff = zeros(max(trajectory_index),3);
interframe_abnormal_list = [];
is_plot_needed = 1 * (enable_plot);
if is_plot_needed == 1
    figure(666)
    set(gcf,'Position',[0 0 2600 1800]);
end
for i = 2 : max(trajectory_index)
    last_single_trajectory_list = find(trajectory_index==i-1);
    if length(last_single_trajectory_list)>100
        last_single_trajectory_list = last_single_trajectory_list(1:end-1);
    end
    last_single_trajectory_time = trajectory_plot_time(last_single_trajectory_list);
    last_single_trajectory_x = trajectory_x(last_single_trajectory_list);
    last_single_trajectory_y = trajectory_y(last_single_trajectory_list);
    last_single_trajectory_theta = trajectory_theta(last_single_trajectory_list);
    last_single_trajectory_v = trajectory_v(last_single_trajectory_list);
    last_single_trajectory_a = trajectory_a(last_single_trajectory_list);
    last_single_trajectory_kappa = trajectory_kappa(last_single_trajectory_list);
    current_single_trajectory_list = find(trajectory_index==i);
    if length(current_single_trajectory_list)>100
        current_single_trajectory_list = current_single_trajectory_list(1:end-1);
    end
    current_single_trajectory_time = trajectory_plot_time(current_single_trajectory_list);
    current_single_trajectory_x = trajectory_x(current_single_trajectory_list);
    current_single_trajectory_y = trajectory_y(current_single_trajectory_list);
    current_single_trajectory_theta = trajectory_theta(current_single_trajectory_list);
    current_single_trajectory_v = trajectory_v(current_single_trajectory_list);
    current_single_trajectory_a = trajectory_a(current_single_trajectory_list);
    current_single_trajectory_kappa = trajectory_kappa(current_single_trajectory_list);
    current_single_trajectory_replan = trajectory_replan(current_single_trajectory_list);
    % seperate stitch from replan
    % replan
    if is_plot_needed == 1
    figure(666)
    plot(current_single_trajectory_time,current_single_trajectory_replan);hold on;grid on
    ylabel("is replan","FontSize",20)
    end
    if  current_single_trajectory_replan(1) == 1
        theta_diff(i,:) = 1e-7*ones(1,3);
        v_diff(i,:) = 1e-7*ones(1,3);
        a_diff(i,:) = 1e-7*ones(1,3);
        kappa_diff(i,:) = 1e-7*ones(1,3);
        xy_diff(i,:) = 1e-7*ones(1,3);
        continue
    end
    trajectory_theta_diff = f_trajectory_diff(last_single_trajectory_theta,last_single_trajectory_time,current_single_trajectory_theta,current_single_trajectory_time);
    for k = 1:length(trajectory_theta_diff)
        trajectory_theta_diff(k) = f_normalize_angle(trajectory_theta_diff(k));
    end
    theta_diff(i,1) = sum(trajectory_theta_diff);
    theta_diff(i,2) = max(trajectory_theta_diff);
    theta_diff(i,3) = mean(trajectory_theta_diff);
    trajectory_v_diff = f_trajectory_diff(last_single_trajectory_v,last_single_trajectory_time,current_single_trajectory_v,current_single_trajectory_time);
    v_diff(i,1) = sum(trajectory_v_diff);
    v_diff(i,2) = max(trajectory_v_diff);
    v_diff(i,3) = mean(trajectory_v_diff);
    trajectory_a_diff = f_trajectory_diff(last_single_trajectory_a,last_single_trajectory_time,current_single_trajectory_a,current_single_trajectory_time);
    a_diff(i,1) = sum(trajectory_a_diff);
    a_diff(i,2) = max(trajectory_a_diff);
    a_diff(i,3) = mean(trajectory_a_diff);
    trajectory_kappa_diff = f_trajectory_diff(last_single_trajectory_kappa,last_single_trajectory_time,current_single_trajectory_kappa,current_single_trajectory_time);
    kappa_diff(i,1) = sum(trajectory_kappa_diff);
    kappa_diff(i,2) = max(trajectory_kappa_diff);
    kappa_diff(i,3) = mean(trajectory_kappa_diff);
    last_single_trajectory_point = [last_single_trajectory_x,last_single_trajectory_y];
    current_single_trajectory_point = [current_single_trajectory_x,current_single_trajectory_y];
    trajectory_xy_diff = f_trajectory_diff(last_single_trajectory_point,last_single_trajectory_time,current_single_trajectory_point,current_single_trajectory_time);
    xy_diff(i,1) = sum(trajectory_xy_diff);
    xy_diff(i,2) = max(trajectory_xy_diff);
    xy_diff(i,3) = mean(trajectory_xy_diff);
end
theta_diff = theta_diff *180/pi;
[max_xy_diff,max_xy_diff_index] = max(xy_diff);
[max_theta_diff,max_theta_diff_index] = max(theta_diff);
[max_v_diff,max_v_diff_index] = max(v_diff);
[max_a_diff,max_a_diff_index] = max(a_diff);
[max_kappa_diff,max_kappa_diff_index] = max(kappa_diff);

abnormal_index_list = [abnormal_index_list;interframe_abnormal_list];
fprintf("\t \t \tinter-frame analysis\n");
name_list = ["xy_diff(m)";"theta_diff(deg)";"v_diff(m/s)";"a_diff(m/s^2)";"kappa_diff"];
max_sum_list = [max_xy_diff(1);max_theta_diff(1);max_v_diff(1);max_a_diff(1);max_kappa_diff(1)];
TP99_sum_list = [f_TP_ratio_value(xy_diff(:,1),0.99);f_TP_ratio_value(theta_diff(:,1),0.99);f_TP_ratio_value(v_diff(:,1),0.99);f_TP_ratio_value(a_diff(:,1),0.99);f_TP_ratio_value(kappa_diff(:,1),0.99)];
max_sum_index_list = [max_xy_diff_index(1);max_theta_diff_index(1);max_v_diff_index(1);max_a_diff_index(1);max_kappa_diff_index(1)];
max_max_list = [max_xy_diff(2);max_theta_diff(2);max_v_diff(2);max_a_diff(2);max_kappa_diff(2)];
TP99_max_list = [f_TP_ratio_value(xy_diff(:,2),0.99);f_TP_ratio_value(theta_diff(:,2),0.99);f_TP_ratio_value(v_diff(:,2),0.99);f_TP_ratio_value(a_diff(:,2),0.99);f_TP_ratio_value(kappa_diff(:,2),0.99)];
max_max_index_list = [max_xy_diff_index(2);max_theta_diff_index(2);max_v_diff_index(2);max_a_diff_index(2);max_kappa_diff_index(2)];
table2 =table(name_list,max_sum_list,TP99_sum_list,max_sum_index_list,max_max_list,TP99_max_list,max_max_index_list,'VariableNames',{'info','max_sum','TP99_sum','max_sum_index','max_max','TP99_max','max_max_index'});
disp(table2);
end
%% kinetic contraints
if enable_singleframe_kinetic_check == 1
tic
% max SWA, max SWA rate, count over SWA, count over SWA rate,count
calculated_SWA_eva = zeros(max(trajectory_index),5);
is_plot_needed = 1 * (enable_plot);
if is_plot_needed ==1
figure(777)
subplot(2,1,1)
xlabel("time[s]","FontSize",20)
ylabel("theta[deg]","FontSize",20)
subplot(2,1,2)
xlabel("time[s]","FontSize",20)
ylabel("theta->SWA[deg]","FontSize",20)
set(gcf,'Position',[0 0 2600 1800]);
end
for i = 1:max(trajectory_index)
    single_trajectory_list = find(trajectory_index==i);
    if length(single_trajectory_list)>100
        single_trajectory_list = single_trajectory_list(1:end-1);
    end
    single_trajectory_x = trajectory_x(single_trajectory_list);
    single_trajectory_y = trajectory_y(single_trajectory_list);
    single_trajectory_theta = trajectory_theta(single_trajectory_list);
    single_trajectory_time = trajectory_plot_time(single_trajectory_list);
    single_trajectory_v = trajectory_v(single_trajectory_list);
    calculated_SWA_deg = zeros(length(single_trajectory_list),1);
    for j = 1:length(single_trajectory_list)-1        
        T = planning_d_relavtive_time;
        current_x = single_trajectory_x(j);
        current_y = single_trajectory_y(j);
        current_theta = single_trajectory_theta(j);
        current_v = single_trajectory_v(j);
        next_x = single_trajectory_x(j+1);
        next_y = single_trajectory_y(j+1);
        next_theta = single_trajectory_theta(j+1);
        syms WSA;
        equation_WSA = @(WSA) current_theta + tan(WSA)*current_v*T/L+current_v*WSA*T/(L*cos(WSA)^2)-next_theta;
        options = optimoptions('fsolve','Display','off');
        calculated_WSA = double(fsolve(equation_WSA,0.0,options));
        calculated_SWA_deg(j) = calculated_WSA *180/pi* transmission_ratio;
        if abs(calculated_SWA_deg(j)) > 620 
            if j >1
                calculated_SWA_deg(j) = calculated_SWA_deg(j-1);
            else
                calculated_SWA_deg(j) = 0;
            end
        end
        if abs(calculated_SWA_deg(j)) > max_SWA
            calculated_SWA_eva(i,3) = calculated_SWA_eva(i,3) + 1;
        end
        if j>1 && abs(calculated_SWA_deg(j)-calculated_SWA_deg(j-1)) > max_SWA_rate * T
            calculated_SWA_eva(i,4) = calculated_SWA_eva(i,4) + 1;
        end
    end
    calculated_SWA_deg(end) = calculated_SWA_deg(end-1);
    calculated_SWA_eva(i,1) = max(abs(calculated_SWA_deg));
    calculated_SWA_eva(i,2) = max(abs(diff(calculated_SWA_deg)/T));
    calculated_SWA_eva(i,5) = length(calculated_SWA_deg);
    if is_plot_needed ==1
    figure(777)
    subplot(2,1,1)
    plot(single_trajectory_time,single_trajectory_theta*180/pi);hold on
    subplot(2,1,2)
    plot(single_trajectory_time,calculated_SWA_deg);hold on
    title("index = "+i,"FontSize",20)
    end
end

[max_max_SWA,max_max_SWA_index] =max(calculated_SWA_eva(:,1));
[max_max_SWA_rate,max_max_SWA_rate_index] =max(calculated_SWA_eva(:,2));
overlimit_SWA_cnt = length(find(calculated_SWA_eva(:,3)>0));
overlimit_SWA_rate_cnt = length(find(calculated_SWA_eva(:,4)>0));
TP99_max_SWA = f_TP_ratio_value(calculated_SWA_eva(:,1),0.99);
TP95_max_SWA = f_TP_ratio_value(calculated_SWA_eva(:,1),0.95);
TP90_max_SWA = f_TP_ratio_value(calculated_SWA_eva(:,1),0.90);
TP99_max_SWA_rate = f_TP_ratio_value(calculated_SWA_eva(:,2),0.99);
TP95_max_SWA_rate = f_TP_ratio_value(calculated_SWA_eva(:,2),0.95);
TP90_max_SWA_rate = f_TP_ratio_value(calculated_SWA_eva(:,2),0.90);
table3 = table(["SWA(deg)";"SWA_rate(deg/s)"],[max_max_SWA;max_max_SWA_rate],[TP99_max_SWA;TP99_max_SWA_rate],[TP95_max_SWA;TP95_max_SWA_rate],[max_max_SWA_index;max_max_SWA_rate_index],...
    [overlimit_SWA_cnt;overlimit_SWA_rate_cnt],'VariableNames',{'theta + v -> SWA','max max','TP99 max','TP95 max','max max idnex','overlimit cnt'});
disp(table3);
toc
end
%% v2xy
% sum, max, index, count
if enable_singleframe_v2xy_check == 1
v2xy_eva = zeros(max(trajectory_index),4);
for i = 1:max(trajectory_index)
    gear_coefficient = 0;
    single_trajectory_list = find(trajectory_index==i);
    if length(single_trajectory_list)>100
        single_trajectory_list = single_trajectory_list(1:end-1);
    end
    single_trajectory_x = trajectory_x(single_trajectory_list);
    single_trajectory_y = trajectory_y(single_trajectory_list);
    single_trajectory_theta = trajectory_theta(single_trajectory_list);
    single_trajectory_time = trajectory_plot_time(single_trajectory_list);
    single_trajectory_v = trajectory_v(single_trajectory_list);
    single_trajectory_a = trajectory_a(single_trajectory_list);
    single_trajectory_gear = trajectory_gear(single_trajectory_list);
    if single_trajectory_gear(1) == 2
       gear_coefficient = -1;
    else
        gear_coefficient = 1;
    end
    calculated_xy_diff = zeros(length(single_trajectory_list),1);
    for j = 1:length(single_trajectory_list)-1        
        T = planning_d_relavtive_time;
        current_x = single_trajectory_x(j);
        current_y = single_trajectory_y(j);
        current_theta = single_trajectory_theta(j);
        current_v = single_trajectory_v(j);
        current_a = single_trajectory_a(j);
        next_x = single_trajectory_x(j+1);
        next_y = single_trajectory_y(j+1);
        next_theta = single_trajectory_theta(j+1);
        average_v = current_v+current_a*0.5*T;
%         calculated_x = current_x + average_v*(cos(current_theta) - sin(current_theta)*current_theta)*T;
%         calculated_y = current_y + average_v*(sin(current_theta) + cos(current_theta)*current_theta)*T;
        calculated_x = current_x + cos(current_theta)*T*average_v*gear_coefficient;
        calculated_y = current_y + sin(current_theta)*T*average_v*gear_coefficient;
        calculated_xy_diff(j) = norm([next_x-calculated_x,next_y-calculated_y]);
    end
    v2xy_eva(i,1) = sum(abs(calculated_xy_diff));
    [v2xy_eva(i,2),v2xy_eva(i,3)] = max(abs(calculated_xy_diff));
    v2xy_eva(i,4) = length(abs(calculated_xy_diff));
end
[max_sum_v2xy,max_sum_v2xy_index] = max(v2xy_eva(:,1));
[max_max_v2xy,max_max_v2xy_index] = max(v2xy_eva(:,2));
abnormal_v2xy_list = find(abs(v2xy_eva(:,2))>0.05);
abnormal_v2xy_cnt = length(abnormal_v2xy_list);
table4 = table("xy_diff(m)",max_sum_v2xy,max_sum_v2xy_index,max_max_v2xy,max_max_v2xy_index,abnormal_v2xy_cnt,'VariableNames',{'xy + theta + v ->xy ','max_sum','max_sum_index','max_max','max_max_index','abnormal cnt'});
disp(table4);
end
%% xy2theta 
if enable_singleframe_xy2theta_check == 1
% sum max count
xy2theta_eva = zeros(max(trajectory_index),1);
is_plot_needed = 1 * (enable_plot);
if is_plot_needed ==1
figure(888)
set(gcf,'Position',[0 0 2600 1800]);

end
for i = 1:max(trajectory_index)
    gear_coefficient = 0;
    single_trajectory_list = find(trajectory_index==i);
    if length(single_trajectory_list)>100
        single_trajectory_list = single_trajectory_list(1:end-1);
    end
    single_trajectory_x = trajectory_x(single_trajectory_list);
    single_trajectory_y = trajectory_y(single_trajectory_list);
    single_trajectory_theta = trajectory_theta(single_trajectory_list);
    single_trajectory_time = trajectory_plot_time(single_trajectory_list);
    single_trajectory_gear = trajectory_gear(single_trajectory_list);
    if single_trajectory_gear(1) == 2
       gear_coefficient = -1;
    else
        gear_coefficient = 1;
    end
    calculated_xy2theta = zeros(length(single_trajectory_list),1);
    for j=1:length(single_trajectory_list)-1
        calculated_xy2theta(j) = f_xy2theta(single_trajectory_x(j),single_trajectory_y(j),single_trajectory_x(j+1),single_trajectory_y(j+1));
        if gear_coefficient == -1
            calculated_xy2theta(j) = f_normalize_angle(calculated_xy2theta(j) + pi);
        end
%         if abs(f_normalize_angle(calculated_xy2theta(j)-single_trajectory_theta(j)))>pi/2
%             calculated_xy2theta(j) = f_normalize_angle(calculated_xy2theta(j) + pi);
%         end

        if norm([single_trajectory_x(j)-single_trajectory_x(j+1),single_trajectory_y(j)-single_trajectory_y(j+1)])<1e-3
            if j>1
                calculated_xy2theta(j) = calculated_xy2theta(j-1);
            else
                calculated_xy2theta(j) = single_trajectory_theta(1);
            end
        end
        
    end
    calculated_xy2theta(end) = calculated_xy2theta(end-1);
    planning2calculated_theta_diff = single_trajectory_theta - calculated_xy2theta;
    for k = 1:length(planning2calculated_theta_diff)
        planning2calculated_theta_diff(k) = f_normalize_angle(planning2calculated_theta_diff(k));
    end
    planning2calculated_theta_diff = planning2calculated_theta_diff*180/pi;
    xy2theta_eva(i,1) = sum(abs(planning2calculated_theta_diff));
    xy2theta_eva(i,2) = max(abs(planning2calculated_theta_diff));
    xy2theta_eva(i,3) = length(planning2calculated_theta_diff);
    if is_plot_needed ==1
    subplot(4,1,1)
    plot(single_trajectory_x,single_trajectory_y)
    xlabel("X[m]","FontSize",20)
    ylabel("Y[m]","FontSize",20)
    grid on
    hold on
    subplot(4,1,2)
    plot(single_trajectory_time,single_trajectory_theta*180/pi)
    xlabel("time[s]","FontSize",20)
    ylabel("planning theta[deg]","FontSize",20)
    hold on
    grid on
    subplot(4,1,3)
    plot(single_trajectory_time,calculated_xy2theta*180/pi)
    xlabel("time[s]","FontSize",20)
    ylabel("xy->theta[deg]","FontSize",20)
    hold on
    grid on
    subplot(4,1,4)
    plot(single_trajectory_time,planning2calculated_theta_diff)
    xlabel("time[s]","FontSize",20)
    ylabel("theta diff[deg]","FontSize",20)
    hold on
    grid on
    end
end
[max_sum_xy2theta_diff,max_sum_xy2theta_diff_index] = max(xy2theta_eva(:,1));
[max_max_xy2theta_diff,max_max_xy2theta_diff_index] = max(xy2theta_eva(:,2));
abnormal_xy2theta_list = find(abs(xy2theta_eva(:,2))>0.2*180/pi);
abnormal_xy2theta_cnt = length(abnormal_xy2theta_list);
abnormal_index_list = [abnormal_index_list;abnormal_xy2theta_list];
table5 = table("theta_diff(deg)",max_sum_xy2theta_diff,max_sum_xy2theta_diff_index,max_max_xy2theta_diff,max_max_xy2theta_diff_index,abnormal_xy2theta_cnt,'VariableNames',{'calculated theta','max_sum','max_sum_idnex','max_max','max_max_index','abnormal cnt'});
disp(table5);
end
%% longitudinal check
if enable_singleframe_v_a_s_check == 1
xy2s_abnormal_list = [];
v2s_abnormal_list = [];
a2v_abnormal_list = [];
% accumulated_sum max 
xy2s_eva = zeros(max(trajectory_index),2);
v2s_eva = zeros(max(trajectory_index),2);
a2v_eva = zeros(max(trajectory_index),2);
is_plot_needed = 1 * (enable_plot);
if is_plot_needed == 1
figure(999)
set(gcf,'Position',[0 0 2600 1800]);
end
for i = 1:max(trajectory_index)
    single_trajectory_list = find(trajectory_index==i);
    T = planning_d_relavtive_time;
    if length(single_trajectory_list)>100
        single_trajectory_list = single_trajectory_list(1:end-1);
    end
    single_trajectory_x = trajectory_x(single_trajectory_list);
    single_trajectory_y = trajectory_y(single_trajectory_list);
    single_trajectory_v = trajectory_v(single_trajectory_list);
    single_trajectory_a = trajectory_a(single_trajectory_list);
    single_trajectory_s = trajectory_s(single_trajectory_list);
    single_trajectory_gear = trajectory_gear(single_trajectory_list);
    single_trajectory_time = trajectory_plot_time(single_trajectory_list);
    single_trajectory_s_from_0 = zeros(length(single_trajectory_list),1);
    single_trajectory_xy2s_from_0 = zeros(length(single_trajectory_list),1);
    single_trajectory_v2s_from_0 = zeros(length(single_trajectory_list),1);
    xy2s_diff = zeros(length(single_trajectory_list),1);
    v2s_diff = zeros(length(single_trajectory_list),1);
    a2v_diff = zeros(length(single_trajectory_list),1);
    for j =1:length(single_trajectory_list)-1
        cur_x = single_trajectory_x(j);
        cur_y = single_trajectory_y(j);
        cur_v = single_trajectory_v(j);
        cur_a = single_trajectory_a(j);
        next_x = single_trajectory_x(j+1);
        next_y = single_trajectory_y(j+1);
        next_v = single_trajectory_v(j+1);
        next_a = single_trajectory_a(j+1);
        distance = norm([cur_x-next_x,cur_y-next_y]);
        v2ds = T * (cur_v+next_v)/2;
        a2dv = T * (cur_a+next_a)/2;
        d_s = single_trajectory_s(j+1)-single_trajectory_s(j);
        d_v = single_trajectory_v(j+1)-single_trajectory_v(j);
        xy2s_diff(j) = distance - d_s;
        v2s_diff(j) = v2ds - d_s;
        a2v_diff(j) = a2dv - d_v;
        single_trajectory_s_from_0(j) = single_trajectory_s_from_0(j) + d_s;
        single_trajectory_xy2s_from_0(j) = single_trajectory_xy2s_from_0(j) + distance;
        single_trajectory_v2s_from_0(j) = single_trajectory_v2s_from_0(j) + v2ds;
    end
    xy2s_eva(i,1) = sum(xy2s_diff);
    xy2s_eva(i,2) = max(abs(xy2s_diff));
    v2s_eva(i,1) = sum(v2s_diff);
    v2s_eva(i,2) = max(abs(v2s_diff));
    a2v_eva(i,1) = sum(a2v_diff);
    a2v_eva(i,2) = max(abs(a2v_diff));
    if  xy2s_eva(i,2)>0.02
        xy2s_abnormal_list = [xy2s_abnormal_list;i];
    end
    if  v2s_eva(i,2)>0.02
        v2s_abnormal_list = [v2s_abnormal_list;i];
    end
    if  a2v_eva(i,2)>0.2
        a2v_abnormal_list = [a2v_abnormal_list;i];
    end
    if is_plot_needed == 1
        figure(999)
        subplot(5,1,1)
        plot(single_trajectory_time(1:end-1),single_trajectory_s_from_0(1:end-1));hold on;grid on;
        xlabel("time[s]","FontSize",20)
        ylabel("s[m]","FontSize",20)
        subplot(5,1,2)
        plot(single_trajectory_time(1:end-1),single_trajectory_xy2s_from_0(1:end-1));hold on;grid on;
        xlabel("time[s]","FontSize",20)
        ylabel("xy->s[m]","FontSize",20)
        subplot(5,1,3)
        plot(single_trajectory_time(1:end-1),xy2s_diff(1:end-1));hold on;grid on;
        xlabel("time[s]","FontSize",20)
        ylabel("xy-s diff[m]","FontSize",20)
        ylim([-0.05 0.05])
        subplot(5,1,4)
        plot(single_trajectory_time(1:end-1),single_trajectory_v2s_from_0(1:end-1));hold on;grid on;
        xlabel("time[s]","FontSize",20)
        ylabel("v->s[m]","FontSize",20)
        subplot(5,1,5)
        plot(single_trajectory_time(1:end-1),v2s_diff(1:end-1));hold on;grid on;
        xlabel("time[s]","FontSize",20)
        ylabel("v-s diff[m]","FontSize",20)
        ylim([-0.05 0.05])
    end
end
[~,max_sum_xy2s_index] = max(abs(xy2s_eva(:,1)));
max_sum_xy2s = xy2s_eva(max_sum_xy2s_index,1);
[max_max_xy2s,max_max_xy2s_index] = max(xy2s_eva(:,2));
[~,max_sum_v2s_index] = max(abs(v2s_eva(:,1)));
max_sum_v2s = v2s_eva(max_sum_v2s_index,1);
[max_max_v2s,max_max_v2s_index] = max(v2s_eva(:,2));
[~,max_sum_a2v_index] = max(abs(a2v_eva(:,1)));
max_sum_a2v = a2v_eva(max_sum_a2v_index,1);
[max_max_a2v,max_max_a2v_index] = max(a2v_eva(:,2));
max_sum_list = [max_sum_xy2s;max_sum_v2s;max_sum_a2v];
max_sum_index_list = [max_sum_xy2s_index;max_sum_v2s_index;max_sum_a2v_index];
max_max_list = [max_max_xy2s;max_max_v2s;max_max_a2v];
max_max_index_list = [max_max_xy2s_index;max_max_v2s_index;max_max_a2v_index];
name_list = ["xy->s(m)";"v->s(m)";"a->v(m/s)"];
abnormal_cnt_list = [length(xy2s_abnormal_list);length(v2s_abnormal_list);length(a2v_abnormal_list)];
table6 = table(name_list, max_sum_list,max_sum_index_list,max_max_list,max_max_index_list,abnormal_cnt_list,...
    'VariableNames',{'diff','max_sum','max_sum_index','max_max','max_max_index','abnormal cnt'});
disp(table6);
abnormal_index_list = [abnormal_index_list;xy2s_abnormal_list;v2s_abnormal_list;a2v_abnormal_list];
end
%%
tabulate_list_1 = tabulate([(1:max(trajectory_index))';abnormal_index_list]);
abnormal_index_list = find(tabulate_list_1(:,2)>1);
end

