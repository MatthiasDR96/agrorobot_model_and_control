function [u] = lqr_control(x, traj)

    % Get index of closest point on trajectory
    dx = x(1) - traj(1,:);
    dy = x(2) - traj(2,:);
    d = sqrt(dx .^ 2 + dy .^ 2);
    [mind, ind] = min(d);
    e = sqrt(mind);
    dxl = traj(1,ind) - x(1);
    dyl = traj(2,ind) - x(2);
        
    A = [1.0, dt, 0.0, 0.0, 0.0; ...
        0.0, 0.0, x(4), 0.0, 0.0; ...
        0.0, 0.0, 1.0, dt, 0.0; ...
        0.0, 0.0, 0.0, 0.0, 0.0;...
        0.0, 0.0, 0.0, 0.0, 1.0];
    
    B = [0.0, 0.0;...
         0.0, 0.0;...
         0.0, 0.0;...
         x(4)/L, 0.0;...
         0.0, dt] ;
     
    % Params
    Q = [8.0 0.0; 0.0 5.0];
    R = eye(1);
    
    % Control gain
    [K, ~, ~] = lqr(A, B, Q, R);
    
    % Error in robot frame
    th_e = mod((atan2(y_e, x_e) + pi),(2 * pi)) - pi;
    
    % State
    x = zeros(5, 1);
    x(1,1) = e;
    x(2,1) = (e - pe) / dt;
    x(3,1) = th_e;
    x(4,1) = (th_e - pth_e) / dt;
    x(5,1) = x(4) - tv;
    
    % Control signal
    u = -K * x
   
end

