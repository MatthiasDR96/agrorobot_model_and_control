function [dx] = kinematics_model(~, x, u, noise)
    
    % State x = [x, y, theta, vx, vy, theta_dot, fi_dot (x4), psi (x4)]
    % Control signal u = [v, omega]
    
    % Load params
    p = load('params.mat');
    dt = p.dt;
    snr = p.snr;
    
    % Add gaussian noise to control value
    if noise
        u = awgn(u, snr, 'measured');
    end
    
    % Calculate required wheel velocities and steering angles
    [fi_dot_req, psi_req] = inverse_kinematics(u(1), u(2));
    
    % Get state variables
    theta = x(3);
    
    % Calculate vehicle velocities in vehicle frame
    [vx, vy, theta_dot] = direct_kinematics(fi_dot_req, psi_req);
    
    % Differential state
    dx = zeros(length(x), 1);
    vx_i = vx * cos(theta) - vy * sin(theta);
    vy_i = vx * sin(theta) + vy * cos(theta);
    dx(1:3) = [vx_i, vy_i, theta_dot];
    dx(4:6) = ([vx; vy; theta_dot] - x(4:6)) / dt;
    dx(7:10) = (fi_dot_req' - x(7:10)) / dt;
    dx(11:14) = (psi_req' - x(11:14)) / dt;
    
    % Add gaussian noise to measured states
    if noise
        dx(1:6) = awgn(dx(1:6), snr, 'measured');
    end
    
end