function [u] = sliding_mode_control(state, traj)

    % Load params
    p = load('params.mat');
    epsilon = p.epsilon;

    % Control params
    k1 = 1;
    k2 = 1;
    k3 = 3;

    % Goal vector in world frame
    X_e = traj(1,1) - state(1);
    Y_e = traj(2,1) - state(2);

    % Goal vector in robot frame
    x_e = cos(state(3)) * X_e + sin(state(3)) * Y_e;
    y_e = -sin(state(3)) * X_e + cos(state(3)) * Y_e;
    
    % Goal angle => Need to be controlled to zero
    theta_d = atan2(y_e, x_e);
    if theta_d > pi
        theta_d = theta_d - 2 * pi;
    elseif theta_d < -pi
        theta_d = theta_d + 2 * pi;
    end

    % Velocities
    v_r = 1;
    omega_r = k3 * theta_d;
    a_r = 0;
    
    % Errors
    x_e = cos(state(3)) * X_e + sin(state(3)) * Y_e;
    y_e = -sin(state(3)) * X_e + cos(state(3)) * Y_e;
    
    % Control commands
    omega = omega_r + ((y_e * a_r) / (1 + ((v_r * y_e) ^ 2))) + (((v_r ^ 2) * sin(theta_d)) / (1 + ((v_r * y_e) ^ 2))) + k2 * min(1.0, max(-1.0, theta_d + atan(v_r * y_e)));
    v = v_r * cos(theta_d) + k1 * min(1.0, max(-1.0, x_e));
    
    % End criterium
    rho = sqrt((traj(1,1) - state(1))^2 + (traj(2,1) - state(2))^2);
    if rho < epsilon
        v = 0;
        omega = 0;
    end
    
    % Compute required wheel speeds and angles
    u = [v omega]';
    
end