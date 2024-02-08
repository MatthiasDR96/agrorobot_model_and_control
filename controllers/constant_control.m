function [u] = constant_control(state, traj)

    % Compute v and omega
    v = 1.0; % m/s
    omega = 1.0; % rad/s
    
    % Compute required wheel speeds and angles
    u = [v, omega]';
    
end