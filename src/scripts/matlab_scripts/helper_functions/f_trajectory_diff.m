function [trajectory_diff] = f_trajectory_diff(last_data,last_time,current_data,current_time)
    max_compare_length = 20;
    data_length = min([length(current_data),max_compare_length]);
    trajectory_diff = [];
    resample_n = 10;
    last_data_resampled = f_resample(last_data,resample_n);
    last_time_resampled = f_resample(last_time,resample_n);
    i = 1;
    last_data_index_pointer = 1;
    while i < data_length+1 && current_time(i) < max(last_time_resampled)
        for j = last_data_index_pointer:length(last_data_resampled)
            time_diff = current_time(i) - last_time_resampled(j);
            if abs(time_diff) < 0.01
                break
            end
        end
        last_data_index_pointer = j;
        diff_at_same_time = norm( current_data(i,:) - last_data_resampled(j,:) );
        trajectory_diff = [trajectory_diff;diff_at_same_time];
        i = i + 1;
    end
end