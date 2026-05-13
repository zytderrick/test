function [lateral_error,station_error] = f_compute_lateral_station_error(current_x,current_y,ref_x,ref_y,ref_heading)
    alpha = ref_heading;
    dx = current_x - ref_x;
    dy = current_y - ref_y;
    lateral_error = cos(alpha) * dy - sin(alpha) * dx;
    station_error = cos(alpha) * dx + sin(alpha) * dy;
end