function resampled_data = f_resample(data,n)
    data_length = length(data);
    resampled_data = [];
    for i = 2:data_length
        start_value = data(i-1,:);
        end_value = data(i,:);
        delta_value = end_value - start_value;
        % specially designed for angle(-pi~pi)
        if size(delta_value,1) == 1 && size(delta_value,2) == 1 && abs(delta_value) > 6 && ...
                abs(abs(start_value)-pi) < 0.1 && abs(abs(end_value)-pi) < 0.1
            delta_value = f_normalize_angle(delta_value);
        end
        step_value = delta_value/n;
        resampled_data = [resampled_data;start_value];
        for j = 1:n-1
            data_ij = start_value + j*step_value;
            resampled_data = [resampled_data;data_ij];
        end
    end
    resampled_data = [resampled_data;data(end,:)];
end