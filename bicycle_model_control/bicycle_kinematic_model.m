function [dx] = bicycle_kinematic_model(x, u)

    % Params
    lr = 0.5;
    lf = 0.5;
    L = lr+lf;
    min_steer = -pi/4;
    max_steer = pi/4;

    % Control signals
    ax = 1;
    delta = u(1);
    
    % Limit steering angles
    if delta >= max_steer
        delta = max_steer;
    end
    if delta <= min_steer
        delta = min_steer;
    end
    
    % Kinematic model
    dx = zeros(4, 1);
    dx(1) = x(4) * cos(x(3));
    dx(2) = x(4) * sin(x(3));
    dx(3) = (x(4) / L) * tan(delta); 
    dx(4) = ax;
    
end