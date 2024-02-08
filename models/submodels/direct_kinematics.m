function [vx, vy, omega] = direct_kinematics(fi_dot,psi)
    
    % Load params
    p = load('params.mat');
    r = p.r;
    CW_1_X = p.CW_1_X;
    CW_2_X = p.CW_2_X;
    CW_3_X = p.CW_3_X;
    CW_4_X = p.CW_4_X;
    CW_1_Y = p.CW_1_Y;
    CW_2_Y = p.CW_2_Y;
    CW_3_Y = p.CW_3_Y;
    CW_4_Y = p.CW_4_Y;
    
    % A matrix
    A = [1 0 1 0 1 0 1 0; 0 1 0 1 0 1 0 1; -CW_1_Y CW_1_X -CW_2_Y CW_2_X -CW_3_Y CW_3_X -CW_4_Y CW_4_X]';
    
    % Direct kinematics
    vx_1 = (fi_dot(1) * r) * cos(psi(1));
    vy_1 = (fi_dot(1) * r) * sin(psi(1));
    vx_2 = (fi_dot(2) * r) * cos(psi(2));
    vy_2 = (fi_dot(2) * r) * sin(psi(2));
    vx_3 = (fi_dot(3) * r) * cos(psi(3));
    vy_3 = (fi_dot(3) * r) * sin(psi(3));
    vx_4 = (fi_dot(4) * r) * cos(psi(4));
    vy_4 = (fi_dot(4) * r) * sin(psi(4));
    B = [vx_1 vy_1 vx_2 vy_2 vx_3 vy_3 vx_4 vy_4]';
    
    % Least squares solver
    sol = lsqr(A, B);
    
    % Compute velocities in inertial frame
    vx = sol(1);
    vy = sol(2);
    omega = sol(3);
    
end

