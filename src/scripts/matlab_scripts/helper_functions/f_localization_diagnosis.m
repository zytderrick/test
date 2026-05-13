%% using physical model to estimate localization and check if localization jump
%%
function [expect_x,expect_y,index,localization_xy_jump_count]=f_localization_diagnosis(x,y,heading,speed,gear)
    index = [];
    control_ts = 0.02;
    data_length = length(x);
    expect_x = zeros(data_length,1);
    expect_y = zeros(data_length,1);
    expect_x(1) = x(1);
    expect_y(1) = y(1);
    localization_xy_jump_count = 0;
    cur2expect_distance_vector = zeros(data_length,1);
    lateral_error_vector = zeros(data_length,1);
    station_error_vector = zeros(data_length,1);
    window_size = 20;
    sum_lateral_error = zeros(data_length-window_size+1,1);
    sum_station_error = zeros(data_length-window_size+1,1);
    for i = 2:data_length
        cur_pos = [x(i) y(i)];
        if gear(i-1) == 2
            expect_x_pos = x(i-1) - control_ts * cos(heading(i-1)) * speed(i-1);
            expect_y_pos = y(i-1) - control_ts * sin(heading(i-1)) * speed(i-1);
        else
            expect_x_pos = x(i-1) + control_ts * cos(heading(i-1)) * speed(i-1);
            expect_y_pos = y(i-1) + control_ts * sin(heading(i-1)) * speed(i-1);
        end
        expect_x(i) = expect_x_pos;
        expect_y(i) = expect_y_pos;
        expect_pos = [expect_x_pos  expect_y_pos];
        cur2expect_distance = norm(cur_pos - expect_pos);
        cur2expect_distance_vector(i) = cur2expect_distance;
        [lat_error,stat_error] = f_compute_lateral_station_error(cur_pos(1),cur_pos(2),expect_x_pos,expect_y_pos,heading(i-1));
        if abs(lat_error) >1
           lat_error = 0;
           stat_error = 0;
        end
        lateral_error_vector(i) = lat_error;
        station_error_vector(i) = stat_error;
        if cur2expect_distance > max(0.05,control_ts *speed(i-1)) && cur2expect_distance < 1e3
            localization_xy_jump_count = localization_xy_jump_count +1;
            index = [index;i];
        end
    end
    for i = window_size : data_length
        sum_lateral_error(i-window_size+1) = sum(lateral_error_vector(i-window_size+1:i));
        sum_station_error(i-window_size+1) = sum(station_error_vector(i-window_size+1:i));
    end
    sum_lateral_error_10_list = find(sum_lateral_error>0.1);
    fprintf("offline:localization_xy_jump_count %d\n",localization_xy_jump_count);
    figure(50)
    plot(x,y,'Color','r','Marker','o','MarkerSize',8);hold on
    plot(expect_x,expect_y,'Color','g','Marker','.','MarkerSize',12);hold on
%     for i = 1:length(sum_lateral_error_10_list)
%         text(x(sum_lateral_error_10_list(i)-window_size),y(sum_lateral_error_10_list(i)-window_size),"X","Color",'k','FontSize',10);
%     end
    axis equal
    grid on
    legend("record pos","expect pos",'Fontsize',20);
    xlabel('X[m]','Fontsize',20)
    ylabel('Y[m]','Fontsize',20)
    figure(51)
    subplot(2,2,1)
    plot(lateral_error_vector);hold on
    ylabel('lateral error','Fontsize',20)
    grid on
    subplot(2,2,2)
    plot(sum_lateral_error);hold on
    ylabel('sum lateral error','Fontsize',20)
    grid on
    subplot(2,2,3)
    plot(station_error_vector);hold on
    ylabel('station error','Fontsize',20)
    grid on
    subplot(2,2,4)
    plot(sum_station_error);hold on
    ylabel('sum station error','Fontsize',20)
    grid on

end