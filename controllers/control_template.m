function [u] = control_template(state, traj)
    
    % Load params
    p = load('params.mat');

    % Compute required wheel speeds and angles
    u = [v omega]';
    
end