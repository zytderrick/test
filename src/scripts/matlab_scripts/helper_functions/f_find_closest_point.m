function index = f_find_closest_point(current_x,current_y,current_time,trajectory_x,trajectory_y,trajectory_time)
    index = 1;
    min_distance = 100;
    for i = 1:1:length(trajectory_x)
        time_gap = abs(current_time-trajectory_time(i));
%         if time_gap > 20
%             continue
%         end
        distance = norm([current_x-trajectory_x(i),current_y-trajectory_y(i)],2);
        if distance < min_distance
            min_distance = distance;
            index = i;
        end
        if min_distance < 0.05
            break;
        end
    end
end