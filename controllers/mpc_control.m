function [u] = mpc_control(state, traj)

    % Load params
    p = load('params.mat');
    dt = p.dt;
    
    % Objective function
    function [J] = obj_func(u, x)
       
        % Get objective values
        Q = eye(2); Q(1,1) = 10; Q(2,2) = 10;
        R = eye(1) * 10;

        % Solve ode
        x_hist = zeros(length(state), length(u));
        x_hist(:,1) = x;
        for i=1:length(u)

            % Compute derivative
            dxdt = dynamics_model_without_motor_models(0, x_hist(:,i), u(:, i), true);

            % Euler update
            x_hist(:, i+1) = x_hist(:, i) + dxdt .* dt;

        end
        
        % Get index of closest point on trajectory
        dx = state(1) - traj(1,:);
        dy = state(2) - traj(2,:);
        d = sqrt(dx .^ 2 + dy .^ 2);
        [~, ind] = min(d);
    
        % Compute cost
        x_err = x_hist(1:2,:) - traj(:, ind:ind+length(u));
        J = sum(sum(x_err' * Q * x_err)) + sum(sum(u' * R * u));      
        
    end

    % Optimize control values
    h = 10; % Optimization horizon
    u0 = zeros(2, h);
    opts = optimoptions('fmincon','Display','off','Algorithm','sqp');
    opt_u = fmincon(@(u) obj_func(u, state), u0, [], [], [], [], [], [], [], opts);
   
    % Get first control signal
    u = opt_u(:, 1);
    
end