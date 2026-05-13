%% calculate SWAdeg according to kappa
%% 
function [steer_wheel_angle] = f_kappa2SWAdeg(ref_kappa)
    load common.mat L transmission_ratio
    steer_wheel_angle = transmission_ratio * 180/pi *atan(L*ref_kappa);
end