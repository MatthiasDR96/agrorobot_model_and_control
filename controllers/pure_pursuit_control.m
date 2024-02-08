function [u] = pure_pursuit_control(state, traj)

    % Load params
    p = load('params.mat');
    epsilon = p.epsilon;
    
    % Control params
    target_speed = 1;
    k = 0.4; % Look ahead distance dependency of linear velocity
    Lfc = 1.0; % Basic look ahead distance
  
    % Get index of closest point on trajectory
    dx = state(1) - traj(1,:);
    dy = state(2) - traj(2,:);
    d = sqrt(dx .^ 2 + dy .^ 2);
    [~, ind] = min(d);
    
    % Calculate look ahead distance (ifo velocity)
    Lf = k * state(4) + Lfc;

    % Search look ahead target point index
    L = 0.0;
    while Lf > L && (ind + 1) < length(traj(1,:))
        dx = traj(1,ind + 1) - traj(1,ind);
        dy = traj(2,ind + 1) - traj(2,ind);
        L = L + sqrt(dx ^ 2 + dy ^ 2);
        ind = ind + 1;
    end
          
    % Get goal point
    if ind < length(traj(1,:))
        tx = traj(1,ind);
        ty = traj(2,ind);
    else
        tx = traj(1,end);
        ty = traj(2,end);
    end    
    plot(tx, ty, 'gO')
    
    % Compute error angle towards goal
    alpha = mod((atan2(ty - state(2), tx - state(1)) - state(3) + pi),(2 * pi)) - pi;
    
    % Compute distance to goal
    ld = sqrt((tx - state(1))^2 + (ty - state(2))^2);
    
    % Compute curvature
    curv = 2 * sin(alpha) / ld;
    
    % Compute omega
    omega = state(4) * curv;
    
    % Reduce v in turns
    v = target_speed / (abs(omega) + 1) ^ 2.5;
    
    % End criterium
    rho = sqrt((traj(1,end) - state(1))^2 + (traj(2,end) - state(2))^2);
    if rho < epsilon
        v = 0;
        omega = 0;
    end
    
    % Compute required wheel speeds and angles
    u = [v omega]';
    
end