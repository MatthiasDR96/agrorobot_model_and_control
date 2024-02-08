function [u] = pid_control(state, traj)

    % Load params
    p = load('params.mat');
    epsilon = p.epsilon;
    
    % PID params
    k_rho = 0.8316; % 6.8292;
    k_alpha = 5.5200; % 9.9832;

    % Distance to goal
    X_e = traj(1,1) - state(1);
    Y_e = traj(2,1) - state(2);
    rho = sqrt(Y_e^2 + X_e^2);
    
    % Calculate angles
    alpha = mod((atan2(Y_e, X_e) - state(3) + pi),(2 * pi)) - pi;
    
    % Calculate omega
    omega = k_alpha * alpha;
    
    % Calculate v
    v = k_rho * rho;
    
    % Driving backwards
    if alpha > pi / 2 || alpha < -pi / 2
        v = -v;
    end
    
    % End criterium
    if rho < epsilon
        v = 0;
        omega = 0;
    end
    
    % Compute required wheel speeds and angles
    u = [v omega]';

end