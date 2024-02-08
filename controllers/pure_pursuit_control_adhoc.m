function [u] = pure_pursuit_control_adhoc(state, traj)

    % Load params
    p = load('params.mat');
    epsilon = p.epsilon;
    
    % Control params
    k = 0.1;
    Lfc = 0.5;
    
    % Lateral control
  
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
    
    % PID angle control
    u = adhoc_control(state, [tx; ty]);
    
    % End criterium
    rho = sqrt((traj(1,end) - state(1))^2 + (traj(2,end) - state(2))^2);
    if rho < epsilon
        u = [0; 0];
    end
    
end