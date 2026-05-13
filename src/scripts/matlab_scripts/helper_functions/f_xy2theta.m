function theta = f_xy2theta(x0,y0,x1,y1)
    theta = atan2(y1-y0,x1-x0);
    theta = f_normalize_angle(theta);
end