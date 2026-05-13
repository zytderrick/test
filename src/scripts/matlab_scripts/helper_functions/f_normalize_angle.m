function [normalized_angle] = f_normalize_angle(angle)
    normalized_angle=mod(angle+pi,2*pi);
    if normalized_angle <0
        normalized_angle = normalized_angle + 2*pi;
    end
    normalized_angle = normalized_angle - pi;
end