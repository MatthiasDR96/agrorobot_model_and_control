function [Fr] = rolling_resistance_calculation(vx, vy, theta_dot, psi)

    % Load params
    p = load('params.mat');
    mu_0 = p.mu_0;
    CW_1_X = p.CW_1_X;
    CW_2_X = p.CW_2_X;
    CW_3_X = p.CW_3_X;
    CW_4_X = p.CW_4_X;
    CW_1_Y = p.CW_1_Y;
    CW_2_Y = p.CW_2_Y;
    CW_3_Y = p.CW_3_Y;
    CW_4_Y = p.CW_4_Y;
    
    % Velocities of the wheel axes in the vehicle reference frame
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
    
    % Rolling resistance coefficient
    mu_1 = mu_0 * v_wheel_1_x_;
    mu_2 = mu_0 * v_wheel_2_x_; 
    mu_3 = mu_0 * v_wheel_3_x_; 
    mu_4 = mu_0 * v_wheel_4_x_; 
    
    % Normal force
    normal_force = normal_force_calculation();
    
    % Rolling resistance force on each wheel
    Fr1 = mu_1 * normal_force(1);
    Fr2 = mu_2 * normal_force(2);
    Fr3 = mu_3 * normal_force(3);
    Fr4 = mu_4 * normal_force(4);
    Fr = [Fr1, Fr2, Fr3, Fr4]';
    
end

