function [slip_ratio, slip_angle] = slip_calculation(vx, vy, theta_dot, omega_wheel, psi)
    
    % "Longitudinal vehicle dynamics using Simulink/Matlab" from Payman
    % Shakouri et. al.
    
    % Load params
    p = load('params.mat');
    r = p.r;
    v_th = p.v_th;
    CW_1_X = p.CW_1_X;
    CW_2_X = p.CW_2_X;
    CW_3_X = p.CW_3_X;
    CW_4_X = p.CW_4_X;
    CW_1_Y = p.CW_1_Y;
    CW_2_Y = p.CW_2_Y;
    CW_3_Y = p.CW_3_Y;
    CW_4_Y = p.CW_4_Y;
    
    %% Calculating wheel axes velocities
    
    % Velocities of the wheel axes in the vehicle frame
    v_wheel_1_x = vx - theta_dot * CW_1_Y;
    v_wheel_1_y = vy + theta_dot * CW_1_X;
    v_wheel_2_x = vx - theta_dot * CW_2_Y;
    v_wheel_2_y = vy + theta_dot * CW_2_X;
    v_wheel_3_x = vx - theta_dot * CW_3_Y;
    v_wheel_3_y = vy + theta_dot * CW_3_X;
    v_wheel_4_x = vx - theta_dot * CW_4_Y;
    v_wheel_4_y = vy + theta_dot * CW_4_X;
    
    % Velocities of the wheel axes in the wheel frame
    v_wheel_1_x_ = v_wheel_1_x * cos(psi(1)) + v_wheel_1_y * sin(psi(1));
    v_wheel_2_x_ = v_wheel_2_x * cos(psi(2)) + v_wheel_2_y * sin(psi(2));
    v_wheel_3_x_ = v_wheel_3_x * cos(psi(3)) + v_wheel_3_y * sin(psi(3));
    v_wheel_4_x_ = v_wheel_4_x * cos(psi(4)) + v_wheel_4_y * sin(psi(4));

    %% Calculate slip ratio
    slip_ratio_1 = 2 * (r * omega_wheel(1) - v_wheel_1_x_) / (v_th + (v_wheel_1_x_^2 / v_th));
    slip_ratio_2 = 2 * (r * omega_wheel(2) - v_wheel_2_x_) / (v_th + (v_wheel_2_x_^2 / v_th));
    slip_ratio_3 = 2 * (r * omega_wheel(3) - v_wheel_3_x_) / (v_th + (v_wheel_3_x_^2 / v_th));
    slip_ratio_4 = 2 * (r * omega_wheel(4) - v_wheel_4_x_) / (v_th + (v_wheel_4_x_^2 / v_th));
    
    %% Calculating slip angles
    slip_angle_1 = psi(1) - atan2(v_wheel_1_y , v_wheel_1_x);
    slip_angle_2 = psi(2) - atan2(v_wheel_2_y , v_wheel_2_x);
    slip_angle_3 = psi(3) - atan2(v_wheel_3_y , v_wheel_3_x);
    slip_angle_4 = psi(4) - atan2(v_wheel_4_y , v_wheel_4_x);
    
    %% Output
    slip_ratio = [slip_ratio_1, slip_ratio_2, slip_ratio_3, slip_ratio_4]';
    slip_angle = [slip_angle_1, slip_angle_2, slip_angle_3, slip_angle_4]';

end

