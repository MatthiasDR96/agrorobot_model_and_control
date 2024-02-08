function [u] = longitudinal_control(state, goal)

    % Pure pursuit
    alpha = atan2(y_g - y, x_g - x) - theta;
    u = 1;
    
end