function [u] = lqr_control(state, traj)

    % Load params
    p = load('params.mat');
    epsilon = p.epsilon;
    
    % Control params
    Q = eye(2);
    R = eye(1);
    
    % A matrix
    A = [0.0 0.0; 0.0 0.0];
    B = [-1.0 0.0; 0.0 -1.0];
    
    % Control gain
    [K, ~, ~] = lqr(A, B, Q, R);
    
    % Error in inertial frame
    X_e = traj(1,1) - state(1);
    Y_e = traj(2,1) - state(2);
    
    % Error in robot frame
    x_e = cos(state(3)) * X_e + sin(state(3)) * Y_e;
    y_e = -sin(state(3)) * X_e + cos(state(3)) * Y_e;
    
    % State
    x = zeros(2, 1);
    x(1,1) = sqrt(x_e ^ 2 + y_e ^ 2);
    x(2,1) = mod((atan2(y_e, x_e) + pi),(2 * pi)) - pi;
    
    % Control signal
    u = -K * x; 
    
    % End criterium
    rho = sqrt((traj(1,1) - state(1))^2 + (traj(2,1) - state(2))^2);
    if rho < epsilon
        u = [0; 0];
    end
    
end

