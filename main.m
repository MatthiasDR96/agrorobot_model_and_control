%% Init

% Clear
clear;
clc;
close all;

% Add paths
addpath('models')
addpath('utils')
    
% Simulation params
model = 'dynamic_without_motor_models'; % Choose between 'dynamic_with_motor_models', 'dynamic_without_motor_models', and 'kinematic'
control = 'pp_adhoc'; % Choose between 'adhoc', 'constant', 'lqr', 'mpc', 'pid', 'pp', 'pp_adhoc', 'pp_pid', 'pp_sm', 'sm'
traj = 'akker2';  % Choose between 'point', 'line', 'sine', 'akker1', 'akker2', 'akker3'
max_sim_time = 5;  % Maximum execution time of the simulation in seconds
noise = true;  % For adding Gaussian noise, put true, otherwise, put false
render = false;  % For rendering the robot, put true, otherwise, put false

%% Simulate
[t, x_hist, traj] = simulate(model, control, traj, max_sim_time, noise, render);

%% Plot

% Plot position in inertial frame
figure(2)
subplot(2,3,1);
plot(t, x_hist(1:3,:))
title('Positions in inertial frame [m]')
xlabel('t [s]')
legend('X','Y', 'theta')

% Plot velocities in robot frame
subplot(2,3,2);
plot(t, x_hist(4:6,:))
title('Velocities in robot frame [m/s]')
xlabel('t [s]')
legend('vx','vy', 'theta_dot')

% Plot wheel velocities
subplot(2,3,4);
plot(t, x_hist(7:10,:))
title('Weel speeds [rad/s]')
xlabel('t [s]')
legend('fidot1','fidot2', 'fidot3','fidot4')

% Plot wheel angles
subplot(2,3,5);
plot(t, x_hist(11:14,:))
title('Weel angles [rad]')
xlabel('t [s]')
legend('psi1','psi2', 'psi3','psi4')

% Plot pos kin and pos dyn
subplot(2,3,[3,6]);
hold on;
plot(x_hist(1, :), x_hist(2, :), 'LineWidth',2);
plot(traj(1,:), traj(2,:));
plot(traj(1,1), traj(2,1),'p', 'MarkerSize',10,'MarkerFaceColor','green');
plot(traj(1,end), traj(2,end),'p', 'MarkerSize',10,'MarkerFaceColor','red');
axis([-5 15 -5 3])
title('Robot position')
xlabel('X [m]')
ylabel('Y [m]')

