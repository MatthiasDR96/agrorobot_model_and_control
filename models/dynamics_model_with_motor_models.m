function [dx] = dynamics_model_with_motor_models(~, x, u, noise)

    % State x = [x, y, theta, vx, vy, theta_dot, fi_dot (x4), 
    % Tel_fi_dot (x4), z_fi_dot (x4), psi (x4), fi_dot_psi (x4), 
    % Tel_psi (x4), z_psi (x4), q_psi (x4)]
    % Control signal u = [v, omega]
    
    % Load params
    p = load('params.mat');
    r = p.r;
    gr = p.gr;
    snr = p.snr;
    Es = p.Es; 
    
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
    fi_dot = x(7:10);
    psi = x(11:14);
    Tel_fi_dot = x(15:18);
    z_fi_dot = x(19:22);
    fi_dot_psi = x(23:26);
    Tel_psi = x(27:30);
    z_psi = x(31:34);
    q_psi = x(35:38);  
    
    % Convert from motor to wheel
    fi_dot_w = fi_dot / gr;
    psi_w = psi;
    
    % Calculate wheel slip and angles
    [slip_ratio, slip_angle] = slip_calculation(vx, vy, theta_dot, fi_dot_w, psi_w);
    
    % Calculate rolling resistance
    Fr = rolling_resistance_calculation(vx, vy, theta_dot, psi);
  
    % Calculate longitudinal tire forces (traction)
    Fx  = traction_force_calculation(slip_ratio);

    % Calculate lateral forces
    Fy  = lateral_force_calculation(slip_angle);

    % Calculate vehicle dynamics
    [ax, ay, theta_dot_dot] = vehicle_dynamics(vx, vy, theta_dot, psi_w, Fx - Fr, Fy);
    
    % Driving motor dynamics
    Tl1 = Fx * r / gr;
    fi_dot_req_m = fi_dot_req' * gr;
    x_m1 = [fi_dot Tel_fi_dot z_fi_dot]';
    u_m1 = [fi_dot_req_m Tl1]';
    dx_m1 = driving_motor_model(x_m1, u_m1);
    
    % Steering motor dynamics
    Tl2 = [Es Es Es Es];
    x_m2 = [psi fi_dot_psi Tel_psi z_psi q_psi]';
    u_m2 = [psi_req; Tl2];
    dx_m2 = steering_motor_model(x_m2, u_m2);
    
    %%  Differential state
    dx = zeros(length(x), 1);
    vx_i = vx * cos(theta) - vy * sin(theta);
    vy_i = vx * sin(theta) + vy * cos(theta);
    dx(1:3) = [vx_i, vy_i, theta_dot];
    dx(4:6) = [ax; ay; theta_dot_dot];
    dx(7:10) = dx_m1(1,:);
    dx(11:14) = dx_m2(1,:);
    dx(15:18) = dx_m1(2,:);
    dx(19:22) = dx_m1(3,:);
    dx(23:26) = dx_m2(2,:);
    dx(27:30) = dx_m2(3,:);
    dx(31:34) = dx_m2(4,:);
    dx(35:38) = dx_m2(5,:);
    
    % Add gaussian noise to measured states
    if noise
        dx(1:6) = awgn(dx(1:6), snr, 'measured');
    end
    
end