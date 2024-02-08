function [u] = pure_pursuit_control(x, traj)

    %% Params
    target_speed = 3;
    k = 0.1;
    Lfc = 0.3;
    Kp = 1;
    L = 1;
    
    %% Longitudianl P-control
    ax = Kp * (target_speed - x(4));
    
    %% Lateral control
  
    % Get index of closest point on trajectory
    dx = x(1) - traj(1,:);
    dy = x(2) - traj(2,:);
    d = sqrt(dx .^ 2 + dy .^ 2);
    [~, ind] = min(d);
    
    % Calculate look ahead distance (ifo velocity)
    Lf = k * x(4) + Lfc;

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
    
    % Calculate heading angle
    alpha = atan2(ty - x(2), tx - x(1)) - x(3);
    
    % Calculate steering angle
    delta = atan2(2.0 * L * sin(alpha) / Lf, 1.0);
            
    %% Control signal
    u = [ax; delta];
    
end