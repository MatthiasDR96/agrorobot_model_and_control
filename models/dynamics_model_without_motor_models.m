function [dx] = dynamics_model_without_motor_models(t, x, u, noise)
    
    % State x = [x, y, theta, vx, vy, theta_dot, fi_dot (x4), psi (x4)]
    % Control signal u = [v, omega]
    
    % Load params
    p = load('params.mat');
    dt = p.dt;
    snr = p.snr;
    
    % Add gaussian noise to control value
    %if noise
        %u = awgn(u, snr, 'measured');
    %end
    
    % Calculate required wheel velocities and steering angles
    [fi_dot_req, psi_req] = inverse_kinematics(u(1), u(2));
    
    % Get state variables
    theta = x(3);
    vx = x(4);
    vy = x(5);
    theta_dot = x(6);
    
    % Motor model
    fi_dot = fi_dot_req;
    psi = psi_req;
    
    % Calculate wheel slip and angles
    [slip_ratio, slip_angle] = slip_calculation(vx, vy, theta_dot, fi_dot, psi);
    
    % Calculate rolling resistance
    Fr = rolling_resistance_calculation(vx, vy, theta_dot, psi);
  
    % Calculate longitudinal tire forces (traction)
    Fx  = traction_force_calculation(slip_ratio);

    % Calculate lateral forces
    Fy  = lateral_force_calculation(slip_angle);

    % Calculate vehicle dynamics
    [ax, ay, theta_dot_dot] = vehicle_dynamics(vx, vy, theta_dot, psi, Fx - Fr, Fy);
    
    %  Differential state
    dx = zeros(length(x), 1);
    vx_i = vx * cos(theta) - vy * sin(theta);
    vy_i = vx * sin(theta) + vy * cos(theta);
    dx(1:3) = [vx_i, vy_i, theta_dot];
    dx(4:6) = [ax; ay; theta_dot_dot];
    dx(7:10) = (fi_dot_req' - x(7:10)) / dt;
    dx(11:14) = (psi_req' - x(11:14)) / dt;
    
    % Add gaussian noise to measured states
    %if noise
        %dx(1:6) = awgn(dx(1:6), snr, 'measured');
    %end
    
end