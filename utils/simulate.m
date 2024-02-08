function [t, x_hist, traj] = simulate(model_, control, traj, max_sim_time, noise, render)
    
    fprintf('Simulation started')
    
    % Load paths
    addpath('matlab_models/submodels')
    addpath('matlab_models')
    addpath('controllers')
    addpath('utils')
    
    % Load params
    p = load('params.mat');
    dt = p.dt;

    % Define model
    if strcmp(model_, 'dynamic_with_motor_models')
        
        % Step size
        dt = dt / 50;
        
        % Initial state [x, y, theta, vx, vy, theta_dot, fi_dot (x4), 
        % Tel_fi_dot (x4), z_fi_dot (x4), psi (x4), fi_dot_psi (x4), 
        % Tel_psi (x4), z_psi (x4), q_psi (x4)]
        x0 = zeros(38, 1);
        
        % Define model
        model = @dynamics_model_with_motor_models;
        
    elseif strcmp(model_, 'dynamic_without_motor_models')
        
        % Initial state [x, y, theta, vx, vy, theta_dot, fi_dot (x4), psi (x4)]
        x0 = zeros(14, 1);
        
        % Define model
        model = @dynamics_model_without_motor_models;
        
    elseif strcmp(model_, 'kinematic')
        
        % Initial state [x, y, theta, vx, vy, theta_dot, fi_dot (x4), psi (x4)]
        x0 = zeros(14, 1);
        
        % Define model
        model = @kinematics_model;
        
    end
    
    % Define controller
    if strcmp(control, 'adhoc')
        controller = @adhoc_control;
    elseif strcmp(control, 'constant')
        controller = @constant_control;
    elseif strcmp(control, 'lqr')
        controller = @lqr_control;
    elseif strcmp(control, 'mpc')
        controller = @mpc_control;
    elseif strcmp(control, 'pid')
        controller = @pid_control;
    elseif strcmp(control, 'sm')
        controller = @sliding_mode_control;
    elseif strcmp(control, 'pp')
        controller = @pure_pursuit_control;
    elseif strcmp(control, 'pp_adhoc')
        controller = @pure_pursuit_control_adhoc;
    elseif strcmp(control, 'pp_pid')
        controller = @pure_pursuit_control_pid;
    elseif strcmp(control, 'pp_sm')
        controller = @pure_pursuit_control_sm;
    end
    
    controller = @mpc_control;
    
    
    % Define trajectory
    traj = trajectory(traj, dt);
    
    % Solve ode
    fprintf('\nSolving ode...')
    tic;
    t = 0:dt:max_sim_time;
    x_hist = zeros(length(x0), length(t));
    x_hist(:,1) = x0;
    for i=1:length(t)-1
        
        % Compute derivative
        dxdt = model(t(i), x_hist(:,i), controller(x_hist(:,i), traj), noise);

        % Euler update
        x_hist(:, i+1) = x_hist(:, i) + dxdt .* dt;
        
    end
    
    fprintf('\nSimulation finished: ')
    toc
        
    % Animate
    if render
        figure();
        filename = ['output/simulation_output_', model_, '.avi'];
        v = VideoWriter(filename,'Motion JPEG AVI');
        v.Quality = 95;
        open(v)
        for i=1:length(t)
            plot_robot(x_hist(1:3, i), x_hist(11:14, i))
            axis([-5 15 -5 3])
            title(['Simulation at time ' num2str(round(t(i))) 's']);
            xlabel('X [m]')
            ylabel('Y [m]')
            grid on;
            drawnow;
            frame = getframe(gcf);
            writeVideo(v,frame);
        end

        % Close video
        close(v);
    end
    
end