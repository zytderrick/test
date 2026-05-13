function [return_value] = f_TP_ratio_value(nums,ratio)
  % ratio from 0 to 1
    if length(nums)<1
        return_value = inf;
        disp("f_TP_ratio_value:not enough data, return inf");
        return
    end
    sorted_nums = sort(nums);
    return_value = sorted_nums(min(floor(length(sorted_nums)*ratio),length(sorted_nums)),1);
end