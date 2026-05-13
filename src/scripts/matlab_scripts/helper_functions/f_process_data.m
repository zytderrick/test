function f_process_data(x,y,steering_angle,curvature,limit,namestr)
raw_data = steering_angle;
%lowpass
filter_order = 400;
frq_sample = 50;
cutoff_frq = 0.1;
D = designfilt('lowpassfir', 'FilterOrder', filter_order, ...
    'CutoffFrequency', cutoff_frq, 'SampleRate', frq_sample);
data_after_low_pass = fftfilt(D,raw_data);

%phase shift
static_data = raw_data;
dynamic_data = data_after_low_pass;
maxlag = filter_order;
frames_shift = AnalysisPhaseShift(static_data,dynamic_data,maxlag);
data_after_shift = dynamic_data;
if frames_shift > 0
    disp("error")
    return
end
frames_shift = -frames_shift;
total_size = length(data_after_shift);
data_after_shift(1:total_size-frames_shift) = data_after_shift(frames_shift+1:total_size);
data_after_shift(total_size-frames_shift+1:total_size) = 0;

%Scenario Analysis: turn
curvature_limit = 0.002;
bflag_turn = zeros(total_size,1);
for i = 1:total_size
    if abs(curvature(i)) >= curvature_limit
        bflag_turn(i) = 1;
    end
end
Index = 1:total_size;Index = Index';
TurnMarkerList = Index(bflag_turn == 1);

%Scenario Analysis: drive straight
straight_deg_limit = 3;
bflag_straight = zeros(total_size,1);
for i = 1:total_size-frames_shift
    if abs(data_after_shift(i)) < straight_deg_limit
        bflag_straight(i) = 1;
    end
end
Index = 1:total_size;Index = Index';
StraightMarkerList = Index(bflag_straight == 1);

%count_through
through_percentage = zeros(size(limit));
bflag_through = zeros(total_size,numel(limit));
for i = 1:length(limit)
    if limit(i) == 0
        [through_percentage(i),bflag_through(:,i)] = CalThroughPercentage(raw_data,data_after_shift,frames_shift);
        continue;
    end
    [through_percentage_positive,bflag_through_positive] = CalThroughPercentage(raw_data,data_after_shift+limit(i),frames_shift);
    [through_percentage_negtive,bflag_through_negtive] = CalThroughPercentage(raw_data,data_after_shift-limit(i),frames_shift);
    through_percentage(i) = through_percentage_positive + through_percentage_negtive;
    bflag_through(:,i) = bflag_through_positive + bflag_through_negtive;
end
steer_angel_limit = limit;
total_through_percentage = through_percentage;

% straight_through_percentage
count_straight_through = bflag_straight'*bflag_through;
straight_through_percentage = count_straight_through'/sum(bflag_straight~=0)*100;
Result = table(steer_angel_limit,total_through_percentage,straight_through_percentage);
% disp(file_name)
disp(Result)

%figure
figure(19)
clf
subplot(2,1,1)
plot(raw_data,"DisplayName","raw_data")
hold on
% plot(data_after_low_pass,"DisplayName","data_after_low_pass")
% plot(data_after_shift,"k--",'LineWidth',1,"DisplayName","data_after_shift_total")
% plot(data_after_shift,"k.--","MarkerIndices",TurnMarkerList,"DisplayName","data_after_shift_turn")
plot(data_after_shift,"k.--","MarkerIndices",StraightMarkerList,"MarkerEdgeColor","r","DisplayName","data_after_shift_straight")
plot(data_after_shift,".","MarkerIndices",TurnMarkerList,"MarkerEdgeColor","y")
hold off
legend("Interpreter","none","Location","best");

% title(title_str,"Interpreter","none")
xlabel("Index")
ylabel("steer_angle(deg)","Interpreter","none")
text_str = cell(length(steer_angel_limit)+1);
text_str(1) = {"steer_angle_limit|  total_through_percentage|  straight_through_percentage"};
for i = 1:length(steer_angel_limit)
    text_str(i+1) = {"           " + num2str(steer_angel_limit(i)) + ...
        "                            " + num2str(total_through_percentage(i)) + "%"+ ...
        "                            " + num2str(straight_through_percentage(i)) + "%"};
end
text(0.05,0.3,text_str,"Units","normalized","Interpreter","none",'Color','k','FontSize',4,"FontWeight","bold");

subplot(2,1,2)
plot(x,y,".-","MarkerIndices",StraightMarkerList,"MarkerEdgeColor","r")
hold on
plot(x,y,".","MarkerIndices",TurnMarkerList,"MarkerEdgeColor","y")
hold on 
plot(x(1),y(1),"k^")
hold off
xlabel("x(m)")
ylabel("y(m)")
% title( file_list(file_num).name(25:end),"Interpreter","none")
set(gcf,'Position',[0 0 3000 1800]);

save_file_name = "steer_through " + namestr +".png";
saveas(gcf,save_file_name)

end

%%
function frames_shift = AnalysisPhaseShift(static_data,dynamic_data,maxlag)
% maxlag = 10;
[correlation,lags]= xcorr(static_data,dynamic_data,maxlag);
phase_shift_result = [lags',correlation];
max_correlation = max(correlation);
index = phase_shift_result(:,2) == max_correlation;
frames_shift = phase_shift_result(index,1);
end

%%
function [through_percentage,bflag_through] = CalThroughPercentage(raw_data,data_after_shift,frames_shift)
i = 1;
cur_err = raw_data(i) - data_after_shift(i);
if cur_err == 0
    is_last_point_equel = 1;
else
    is_last_point_equel = 0;
end
total_size = length(raw_data);
bflag_through = zeros(total_size,1);
count_through = 0;
for i = 2:total_size-frames_shift
    cur_err = raw_data(i) - data_after_shift(i);
    last_err = raw_data(i-1) - data_after_shift(i-1);
    if is_last_point_equel && cur_err ~= 0
        count_through = count_through + 1;
        bflag_through(i-1) = 1;
        is_last_point_equel = 0;        
    end
    if cur_err == 0
        is_last_point_equel = 1;
    end
    if cur_err*last_err < 0
        count_through = count_through + 1;
        bflag_through(i) = 1;
    end
end
through_percentage = count_through/(total_size-frames_shift)*100;
end