%% lqr csv file analysis
close all;
clear;
%%
lqr_data = readmatrix("steer_log_lqr_2023-04-10_192220.csv");
% auto_mode: %d, time: %.4f, x: %.4f, y: %.4f, heading: %.4f, vel: %.4f, curr_index: %d, 
% "ref_heading: %.4f, lateral_error: %.4f, lateral_error_rate: %.4f,"
% "heading_error: %.4f, heading_error_rate: %.4f,  curvature: %.4f,"
% "send_steer_angle: %.4f, chassis_steer_angle: %.4f,"
% "feedforwardterm: %.4f, feedbackterm: %.4f,"
% "lateral_contribution: %.4f, lateral_rate_contribution: %.4f, "
% "heading_contribution: %.4f, heading_rate_contribution: %.4f, "
% "lateral_evaluation: %.4f, heading_evaluation: %.4f,"
% "record_x: %.4f, record_y: %.4f, record_head: %.4f, num_iterations: %d,"
% "chassis_steer_percentage: %.4f",
lqr_auto_mode = lqr_data(:,1);
lqr_time = lqr_data(:,2);
lqr_x = lqr_data(:,3);
lqr_y = lqr_data(:,4);
lqr_heading = lqr_data(:,5);
lqr_vel = lqr_data(:,6);
lqr_curr_index = lqr_data(:,7);
lqr_ref_heading = lqr_data(:,8);
lqr_lateral_error = lqr_data(:,9);
lqr_lateral_error_rate = lqr_data(:,10);
lqr_heading_error = lqr_data(:,11);
lqr_heading_error_rate = lqr_data(:,12);
lqr_curvature = lqr_data(:,13);
lqr_send_steer_angle = lqr_data(:,14);
lqr_chassis_steer_angle = lqr_data(:,15);
lqr_feedforwardterm = lqr_data(:,16);
lqr_feedbackterm = lqr_data(:,17);
lqr_lateral_contribution = lqr_data(:,18);
lqr_lateral_rate_contribution = lqr_data(:,19);
lqr_heading_contribution = lqr_data(:,20);
lqr_heading_rate_contribution= lqr_data(:,21);
lqr_lateral_evaluation = lqr_data(:,22);
lqr_heading_evaluation = lqr_data(:,23);
lqr_expected_x = lqr_data(:,24);
lqr_expected_y = lqr_data(:,25);
lqr_record_x = lqr_data(:,26);
lqr_record_y = lqr_data(:,27);
lqr_record_head = lqr_data(:,28);
lqr_num_iterations = lqr_data(:,29);

%% process
lqr_data_length =length(lqr_data);
lqr_plot_time = lqr_time(:,1) -lqr_time(1,1);
lqr_heading_deg = lqr_heading .*180/pi;
lqr_heading_error_deg = lqr_heading_error .*180/pi;

%% driving & not driving
lqr_driving_x =[];
lqr_driving_y =[];
lqr_driving_time = [];
lqr_driving_heading = [];
lqr_driving_lateral_error = [];
lqr_driving_lateral_error_rate = [];
lqr_driving_heading_error = [];
lqr_driving_heading_error_rate = [];
lqr_driving_feedforwardterm = [];
lqr_driving_feedbackterm = [];
lqr_driving_send_steer_angle = [];
lqr_driving_chassis_steer_angle = [];

lqr_notdriving_x =[];
lqr_notdriving_y =[];
lqr_notdriving_time = [];
lqr_notdriving_heading = [];
lqr_notdriving_lateral_error = [];
lqr_notdriving_lateral_error_rate = [];
lqr_notdriving_heading_error = [];
lqr_notdriving_heading_error_rate = [];
lqr_notdriving_feedforwardterm = [];
lqr_notdriving_feedbackterm = [];
lqr_notdriving_send_steer_angle = [];
lqr_notdriving_chassis_steer_angle = [];
for i = 1:lqr_data_length
    if lqr_auto_mode(i)==0
        lqr_notdriving_x = [lqr_notdriving_x;lqr_x(i)];
        lqr_notdriving_y = [lqr_notdriving_y;lqr_y(i)];
        lqr_notdriving_time =[lqr_notdriving_time;lqr_plot_time(i)];
        lqr_notdriving_heading = [lqr_notdriving_heading;lqr_heading(i)];
        lqr_notdriving_lateral_error = [lqr_notdriving_lateral_error;lqr_lateral_error(i)];
        lqr_notdriving_lateral_error_rate = [lqr_notdriving_lateral_error_rate;lqr_lateral_error_rate(i)];
        lqr_notdriving_heading_error = [lqr_notdriving_heading_error;lqr_heading_error(i)];
        lqr_notdriving_heading_error_rate = [lqr_notdriving_heading_error_rate;lqr_heading_error_rate(i)];
        lqr_notdriving_feedforwardterm = [lqr_notdriving_feedforwardterm;lqr_feedforwardterm(i)];
        lqr_notdriving_feedbackterm = [lqr_notdriving_feedbackterm;lqr_feedbackterm(i)];
        lqr_notdriving_send_steer_angle = [lqr_notdriving_send_steer_angle;lqr_send_steer_angle(i)];
        lqr_notdriving_chassis_steer_angle = [lqr_notdriving_chassis_steer_angle;lqr_chassis_steer_angle(i)];
    else
        lqr_driving_x = [lqr_driving_x;lqr_x(i)];
        lqr_driving_y = [lqr_driving_y;lqr_y(i)];
        lqr_driving_time =[lqr_driving_time;lqr_plot_time(i)];
        lqr_driving_heading = [lqr_driving_heading;lqr_heading(i)];
        lqr_driving_lateral_error = [lqr_driving_lateral_error;lqr_lateral_error(i)];
        lqr_driving_lateral_error_rate = [lqr_driving_lateral_error_rate;lqr_lateral_error_rate(i)];
        lqr_driving_heading_error = [lqr_driving_heading_error;lqr_heading_error(i)];
        lqr_driving_heading_error_rate = [lqr_driving_heading_error_rate;lqr_heading_error_rate(i)];
        lqr_driving_feedforwardterm = [lqr_driving_feedforwardterm;lqr_feedforwardterm(i)];
        lqr_driving_feedbackterm = [lqr_driving_feedbackterm;lqr_feedbackterm(i)];
        lqr_driving_send_steer_angle = [lqr_driving_send_steer_angle;lqr_send_steer_angle(i)];
        lqr_driving_chassis_steer_angle = [lqr_driving_chassis_steer_angle;lqr_chassis_steer_angle(i)];
    end
end

%%
heading_contribution_diff = [0;diff(lqr_heading_contribution)];
twist_index = [];
for i=2:lqr_data_length
    if heading_contribution_diff(i)*heading_contribution_diff(i-1)< 0
        twist_index =[twist_index;i];
    end
end

%%
figure(1)
plot(lqr_driving_x,lqr_driving_y,'Marker','o','Markersize',5,'Color','r');hold on
plot(lqr_notdriving_x,lqr_notdriving_y,'Marker','o','Markersize',5,'Color','b');hold on

grid on
xlabel("x")
ylabel("y")
axis equal
set(gcf,'Position',[0 0 2600 1600]);
%%
% figure(2)
% subplot(4,1,1)
% plot(lqr_driving_time,lqr_driving_lateral_error,'Marker','o','Markersize',5,'Color','r');hold on
% plot(lqr_notdriving_time,lqr_notdriving_lateral_error,'Marker','o','Markersize',5,'Color','b');hold on
% xlabel("time","FontSize",20)
% ylabel("lateral error","FontSize",20)
% 
% subplot(4,1,2)
% plot(lqr_driving_time,lqr_driving_lateral_error_rate,'Marker','o','Markersize',5,'Color','r');hold on
% plot(lqr_notdriving_time,lqr_notdriving_lateral_error_rate,'Marker','o','Markersize',5,'Color','b');hold on
% xlabel("time","FontSize",20)
% ylabel("lateral error rate","FontSize",20)
% 
% subplot(4,1,3)
% plot(lqr_driving_time,lqr_driving_heading_error,'Marker','o','Markersize',5,'Color','r');hold on
% plot(lqr_notdriving_time,lqr_notdriving_heading_error,'Marker','o','Markersize',5,'Color','b');hold on
% xlabel("time","FontSize",20)
% ylabel("heading error","FontSize",20)
% 
% subplot(4,1,4)
% plot(lqr_driving_time,lqr_driving_heading_error_rate,'Marker','o','Markersize',5,'Color','r');hold on
% plot(lqr_notdriving_time,lqr_notdriving_heading_error_rate,'Marker','o','Markersize',5,'Color','b');hold on
% xlabel("time","FontSize",20)
% ylabel("heading error rate","FontSize",20)
% set(gcf,'Position',[0 0 2600 1600]);
%%
figure(3)
subplot(2,1,1)
plot(lqr_driving_time,lqr_driving_heading_error*180/pi,'Marker','o','Markersize',3,'Color','r');hold on
plot(lqr_notdriving_time,lqr_notdriving_heading_error*180/pi,'Marker','o','Markersize',3,'Color','b');hold on
% if size(twist_index,1)>0
%    for i = 1 :size(twist_index,1)
%        text(lqr_plot_time(twist_index(i)),lqr_heading_error_deg(twist_index(i)),['x'],'color','k','FontSize',20);
%        hold on
%    end
% end
grid on
xlabel("time","FontSize",20)
ylabel("heading error[deg]","FontSize",20)

subplot(2,1,2)
plot(lqr_driving_time,lqr_driving_heading*180/pi,'Marker','o','Markersize',3,'Color','r');hold on
% plot(lqr_notdriving_time,lqr_notdriving_heading*180/pi,'Marker','o','Markersize',3,'Color','b');hold on
plot(lqr_plot_time,lqr_ref_heading*180/pi,'Marker','o','Markersize',3,'Color','g')
legend("heading","ref heading")
% if size(twist_index,1)>0
%    for i = 1 :size(twist_index,1)
%        text(lqr_plot_time(twist_index(i)),lqr_heading_deg(twist_index(i)),['x'],'color','k','FontSize',20);
%        hold on
%    end
% end
grid on
xlabel("time","FontSize",20)
ylabel("heading","FontSize",20)

% subplot(3,1,3)
% plot(lqr_driving_time,lqr_driving_feedforwardterm,'Marker','o','Markersize',3,'Color','r');hold on
% plot(lqr_notdriving_time,lqr_notdriving_feedforwardterm,'Marker','o','Markersize',3,'Color','b');hold on
% xlabel("time")
% ylabel("feedforwardterm")

% windowsize = 100;
% coeff = ones(1, windowsize)/windowsize;
% filt_delay = floor(windowsize/2);
% filtered_ref_heading = filter(coeff, 1, lqr_ref_heading);
% filtered_ref_heading(1:end-filt_delay+1) = filtered_ref_heading(filt_delay:end);
% filtered_ref_heading(end-filt_delay+1:end) = zeros(filt_delay,1);
% 
% lqr_heading_filtered_error = lqr_heading - filtered_ref_heading;
% figure(12)
% subplot(2,1,1)
% plot(lqr_plot_time,lqr_ref_heading*180/pi,'Marker','o','Markersize',3,'Color','r');hold on
% plot(lqr_plot_time,filtered_ref_heading*180/pi,'Marker','o','Markersize',3,'Color','g');hold on
% 
% subplot(2,1,2)
% plot(lqr_plot_time,(lqr_heading-lqr_ref_heading)*180/pi,'Marker','o','Markersize',3,'Color','r');hold on
% plot(lqr_plot_time,(lqr_heading-filtered_ref_heading)*180/pi,'Marker','o','Markersize',3,'Color','g');hold on
% ylim([-1 1])
%%
% dheading_error = [0;diff(lqr_heading_error)];
% dfeedforwardterm = [0;diff(lqr_feedforwardterm)];
% dheading_error_index = find(abs(dheading_error)>0.001);
% dfeedforwardterm_index = find(abs(dfeedforwardterm)>3);
% figure(4)
% subplot(3,1,1)
% plot(lqr_plot_time,lqr_auto_mode);hold on
% xlim([25 40])
% xlabel("time","FontSize",20)
% ylabel("driving mode","FontSize",20)
% 
% subplot(3,1,2)
% plot(lqr_plot_time,dheading_error);hold on
% % if size(dheading_error_index,1)>0
% %    for i = 1 :size(dheading_error_index,1)
% %        text(lqr_plot_time(dheading_error_index(i)),dheading_error(dheading_error_index(i)),...
% %            ['x'],'color','r','FontSize',20);
% %        hold on
% %    end
% % end
% xlim([25 40])
% xlabel("time","FontSize",20)
% ylabel("diff heading error","FontSize",20)
% 
% subplot(3,1,3)
% plot(lqr_plot_time,dfeedforwardterm);hold on
% % if size(dfeedforwardterm_index,1)>0
% %    for i = 1 :size(dfeedforwardterm_index,1)
% %        text(lqr_plot_time(dfeedforwardterm_index(i)),dfeedforwardterm(dfeedforwardterm_index(i)),...
% %            ['x'],'color','r','FontSize',20);
% %        hold on
% %    end
% % end
% xlim([25 40])
% xlabel("time","FontSize",20)
% ylabel("diff feedforwardterm","FontSize",20)
% set(gcf,'Position',[0 0 2600 1600]);
%% 

lpfilter1 = designfilt('lowpassfir', 'FilterOrder', 5, ...
    'CutoffFrequency',0.1 , 'SampleRate', 50);
filtered_steer_angle = fftfilt(lpfilter1,lqr_chassis_steer_angle);

chassis_steer_angle_diff = zeros(lqr_data_length,1);
chassis_steer_angle_diff_index = [];
for i = 2:lqr_data_length
    chassis_steer_angle_diff(i) = filtered_steer_angle(i) - filtered_steer_angle(i-1);
    if abs(chassis_steer_angle_diff(i)) > 3 && lqr_auto_mode(i) == 1
        chassis_steer_angle_diff_index = [chassis_steer_angle_diff_index;i];
    end
end
chassis_steer_angle_diff2 = zeros(lqr_data_length,1);
chassis_steer_angle_diff2_index = [];
for i = 2:lqr_data_length
    chassis_steer_angle_diff2(i) = chassis_steer_angle_diff(i) - chassis_steer_angle_diff(i-1);
    if abs(chassis_steer_angle_diff2(i)) > 1 && lqr_auto_mode(i) == 1
        chassis_steer_angle_diff2_index = [chassis_steer_angle_diff2_index;i];
    end
end
chassis_steer_angle_diff3 = zeros(lqr_data_length,1);
chassis_steer_angle_diff3_index = [];
for i = 2:lqr_data_length
    chassis_steer_angle_diff3(i) = chassis_steer_angle_diff2(i) - chassis_steer_angle_diff2(i-1);
    if abs(chassis_steer_angle_diff3(i)) > 0.5 && lqr_auto_mode(i) == 1
        chassis_steer_angle_diff3_index = [chassis_steer_angle_diff3_index;i];
    end
end
feedforward_diff = zeros(lqr_data_length,1);
feedforward_diff_index = [];
for i = 2:lqr_data_length
    feedforward_diff(i) = lqr_feedforwardterm(i) - lqr_feedforwardterm(i-1);
    if abs(feedforward_diff(i)) > 4 && lqr_auto_mode(i) == 1
        feedforward_diff_index = [feedforward_diff_index;i];
    end
end
feedforward_diff2 = zeros(lqr_data_length,1);
feedforward_diff2_index = [];
for i = 2:lqr_data_length
    feedforward_diff2(i) = feedforward_diff(i) - feedforward_diff(i-1);
    if abs(feedforward_diff2(i)) > 1 && lqr_auto_mode(i) == 1
        feedforward_diff2_index = [feedforward_diff2_index;i];
    end
end

feedback_diff = zeros(lqr_data_length,1);
feedback_diff_index = [];
for i = 2:lqr_data_length
    feedback_diff(i) = lqr_feedbackterm(i) - lqr_feedbackterm(i-1);
    if abs(feedback_diff(i)) > 4 && lqr_auto_mode(i) == 1
        feedback_diff_index = [feedback_diff_index;i];
    end
end
% disp("chassis_steer_angle_diff_index:"+length(chassis_steer_angle_diff_index));
% disp("chassis_steer_angle_diff2_index:"+length(chassis_steer_angle_diff2_index));
% disp("chassis_steer_angle_diff3_index:"+length(chassis_steer_angle_diff3_index));
% disp("feedforward_diff_index:"+length(feedforward_diff_index));
% disp("feedforward_diff2_index:"+length(feedforward_diff2_index));
% disp("feedback_diff_index:"+length(feedback_diff_index));
%%
% figure(5)
% subplot(3,1,1)
% plot(lqr_plot_time,chassis_steer_angle_diff,'Marker','o','Markersize',1,'Color','b')
% xlabel("time")
% ylabel("angle diff")
% hold on 
% 
% subplot(3,1,2)
% plot(lqr_plot_time,chassis_steer_angle_diff2,'Marker','o','Markersize',1,'Color','b')
% xlabel("time")
% ylabel("angle diff2")
% hold on 
% 
% subplot(3,1,3)
% plot(lqr_plot_time,chassis_steer_angle_diff3,'Marker','o','Markersize',1,'Color','b')
% xlabel("time")
% ylabel("angle diff3")
% hold on 
% set(gcf,'Position',[0 0 2600 1600]);
%%
figure(6)
subplot(3,1,1)
plot(lqr_driving_time,lqr_driving_feedforwardterm,'Marker','o','Markersize',3,'Color','r');hold on
plot(lqr_notdriving_time,lqr_notdriving_feedforwardterm,'Marker','o','Markersize',3,'Color','b');hold on
% if size(chassis_steer_angle_diff_index,1)>0
%    for i = 1 :size(chassis_steer_angle_diff_index,1)
%        text(lqr_plot_time(chassis_steer_angle_diff_index(i)),lqr_feedforwardterm(chassis_steer_angle_diff_index(i)),...
%            ['x'],'color','g','FontSize',20);
%        hold on
%    end
% end
if size(chassis_steer_angle_diff2_index,1)>0
   for i = 1 :size(chassis_steer_angle_diff2_index,1)
       text(lqr_plot_time(chassis_steer_angle_diff2_index(i)),lqr_feedforwardterm(chassis_steer_angle_diff2_index(i)),...
           ['x'],'color','c','FontSize',20);
       hold on
   end
end
if size(feedforward_diff_index,1)>0
   for i = 1 :size(feedforward_diff_index,1)
       text(lqr_plot_time(feedforward_diff_index(i)),lqr_feedforwardterm(feedforward_diff_index(i)),['x'],'color','k','FontSize',20);
       hold on
   end
end

xlabel("time","FontSize",20)
ylabel("feedforwardterm","FontSize",20)

subplot(3,1,2)
plot(lqr_driving_time,lqr_driving_feedbackterm,'Marker','o','Markersize',3,'Color','r');hold on
plot(lqr_notdriving_time,lqr_notdriving_feedbackterm,'Marker','o','Markersize',3,'Color','b');hold on
% if size(feedback_diff_index,1)>0
%    for i = 1 :size(feedback_diff_index,1)
%        text(lqr_plot_time(feedback_diff_index(i)),lqr_feedbackterm(feedback_diff_index(i)),['x'],'color','b','FontSize',20);
%        hold on
%    end
% end
xlabel("time","FontSize",20)
ylabel("feedbackterm","FontSize",20)

subplot(3,1,3)
plot(lqr_driving_time,lqr_driving_chassis_steer_angle,'Marker','o','Markersize',3,'Color','r');hold on
plot(lqr_notdriving_time,lqr_notdriving_chassis_steer_angle,'Marker','o','Markersize',3,'Color','b');hold on
plot(lqr_plot_time,filtered_steer_angle,'Marker','.')
% if size(chassis_steer_angle_diff_index,1)>0
%    for i = 1 :size(chassis_steer_angle_diff_index,1)
%        text(lqr_plot_time(chassis_steer_angle_diff_index(i)),filtered_steer_angle(chassis_steer_angle_diff_index(i)),['x'],'color','g','FontSize',20);
%        hold on
%    end
% end
if size(chassis_steer_angle_diff2_index,1)>0
   for i = 1 :size(chassis_steer_angle_diff2_index,1)
       text(lqr_plot_time(chassis_steer_angle_diff2_index(i)),filtered_steer_angle(chassis_steer_angle_diff2_index(i)),['x'],'color','c','FontSize',20);
       hold on
   end
end
if size(feedforward_diff_index,1)>0
   for i = 1 :size(feedforward_diff_index,1)
       text(lqr_plot_time(feedforward_diff_index(i)),filtered_steer_angle(feedforward_diff_index(i)),['x'],'color','k','FontSize',20);
       hold on
   end
end
% if size(feedback_diff_index,1)>0
%    for i = 1 :size(feedback_diff_index,1)
%        text(lqr_plot_time(feedback_diff_index(i)),filtered_steer_angle(feedback_diff_index(i)),['x'],'color','b','FontSize',20);
%        hold on
%    end
% end
xlabel("time","FontSize",20)
ylabel("chassis steer angle","FontSize",20)
set(gcf,'Position',[0 0 2600 1600]);
%%
windowsize = 20;
coeff = ones(1, windowsize)/windowsize;
filt_delay = windowsize/2;
filtered_ff = filter(coeff, 1, lqr_feedforwardterm);
filtered_ff(1:end-filt_delay+1) = filtered_ff(filt_delay:end);
filtered_ff(end-filt_delay+1:end) = zeros(filt_delay,1);

figure(7)
subplot(3,1,1)
plot(lqr_plot_time,lqr_auto_mode);hold on
xlabel("time","FontSize",20)
ylabel("driving mode","FontSize",20)


subplot(3,1,2)
plot(lqr_plot_time,lqr_feedforwardterm,'Marker','o','Markersize',3,'Color','r');hold on
plot(lqr_plot_time,filtered_ff,'Marker','o','Markersize',3,'Color','b');hold on 
legend("origin","smoothed")
xlabel("time","FontSize",20)
ylabel("feedforward","FontSize",20)

subplot(3,1,3)
plot(lqr_driving_time,lqr_driving_send_steer_angle,'Marker','o','Markersize',3,'Color','r');hold on
plot(lqr_notdriving_time,lqr_notdriving_send_steer_angle,'Marker','o','Markersize',3,'Color','b');hold on
plot(lqr_plot_time,filtered_ff+lqr_feedbackterm,'Marker','o','Markersize',3,'Color','g');hold on 
set(gcf,'Position',[0 0 2600 1600]);
% legend("origin","----","smoothed")
xlabel("time","FontSize",20)
ylabel("send steer angle","FontSize",20)
ylim([-250 250])


%%
% figure(8)
% plot(lqr_driving_time,lqr_driving_chassis_steer_angle,'Marker','o','Markersize',3,'Color','r');hold on
% plot(lqr_notdriving_time,lqr_notdriving_chassis_steer_angle,'Marker','o','Markersize',3,'Color','b');hold on
% plot(lqr_plot_time,filtered_steer_angle,'Marker','.')
% set(gcf,'Position',[0 0 2600 1600]);
% legend("origin","----","filtered")
% xlabel("time","FontSize",20)
% ylabel("chassis steer angle","FontSize",20)
% ylim([-150 150])

%%

figure(9)

% plot(lqr_plot_time,filtered_ff+lqr_feedbackterm,'Marker','o','Markersize',3,'Color','g');hold on 
plot(lqr_plot_time,lqr_feedbackterm,'Marker','_','Markersize',3,'Color','m');hold on 
plot(lqr_plot_time,lqr_lateral_contribution+lqr_heading_contribution,'Marker','o','Markersize',1,'Color','b','LineStyle','-');hold on 
set(gcf,'Position',[0 0 2600 1600]);
% legend("origin","----","smoothed","feedback")
legend("lqr feedbackterm","heading+lateral")
xlabel("time","FontSize",20)
ylabel("send steer angle","FontSize",20)
ylim([-250 250])
%%

figure(10)
plot(lqr_plot_time,lqr_feedbackterm,'Marker','o','Markersize',5,'Color','g');hold on 
plot(lqr_plot_time,lqr_heading_contribution,'Color','r','LineStyle','--');hold on 
plot(lqr_plot_time,lqr_lateral_contribution,'Color','b','LineStyle','--');hold on 
grid on
% if size(twist_index,1)>0
%    for i = 1 :size(twist_index,1)
%        text(lqr_plot_time(twist_index(i)),lqr_heading_contribution(twist_index(i)),['x'],'color','k','FontSize',20);
%        hold on
%    end
% end
set(gcf,'Position',[0 0 2600 1600]);
legend("fb","heading","lateral")
xlabel("time","FontSize",20)
ylabel("send steer angle","FontSize",20)
% xlim([110 130])
ylim([-50 50])

windowsize = 20;
coeff = ones(1, windowsize)/windowsize;
filt_delay = windowsize/2;
filtered_heading_error = filter(coeff, 1, lqr_heading_error);
filtered_heading_error(1:end-filt_delay+1) = filtered_heading_error(filt_delay:end);
filtered_heading_error(end-filt_delay+1:end) = zeros(filt_delay,1);
% filtered_heading_error = fftfilt(lpfilter1,lqr_heading_error);

%%
figure(11)
plot(lqr_plot_time,lqr_send_steer_angle,'Marker','o');hold on
plot(lqr_plot_time,lqr_chassis_steer_angle,'Marker','o');hold on 
legend("send steer","chassis steer")
grid on
xlabel("time","FontSize",20)
ylabel("send steer angle","FontSize",20)

%%
