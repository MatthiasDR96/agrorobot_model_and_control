% Clear
clc
clear
close all

% Ode solver
tspan = [0 10];
x0 = [0;0;0;0]; %[x, y, theta, v]
[t, x] = ode45(@(t,x) odefun(t,x), tspan, x0);

% Plot traj
traj = trajectory('sine');
figure()
plot(traj(1,:), traj(2,:), '.r');
hold on;

% Plot
axlim = [min(x(:,1)) max(x(:,1))  min(x(:,2)) max(x(:,2))];
grid on;
for i=1:length(t)
    state = [x(i,1), x(i,2), x(i,3)];
    plot_robot(state);
    plot(x(i,1), x(i,2), '.b');
    drawnow;
    pause(0.1);
end

%% Ode function

function [dx] = odefun(t, x)
    
    % Get trajectory
    traj = trajectory('sine');
    
    % Get control signal
    u = lqr_control(x, traj);
    
    % Compute next state using dynamics
    dx = bicycle_kinematic_model(x, u);
    
end

