function [] = plot_robot(state, psi)

    % Load params
    p = load('params.mat');
    
    % Extract params
    CW_1_X = p.CW_1_X;
    CW_2_X = p.CW_2_X;
    CW_3_X = p.CW_3_X;
    CW_4_X = p.CW_4_X;
    CW_1_Y = p.CW_1_Y;
    CW_2_Y = p.CW_2_Y;
    CW_3_Y = p.CW_3_Y;
    CW_4_Y = p.CW_4_Y;
    
    % Vehicle coordinates
    vehicle_points = [CW_1_X CW_1_Y 1; CW_2_X CW_2_Y 1; CW_4_X CW_4_Y 1; CW_3_X CW_3_Y 1; CW_1_X CW_1_Y 1;]';

    % Plot center
    plot(state(1), state(2), 'b.', 'MarkerSize', 10);
    quiver(state(1), state(2), cos(state(3)), sin(state(3)), 0.5, 'color',[1 1 0]);
    
    % Convert robot point to new pos
    T = [cos(state(3)) -sin(state(3)) state(1); sin(state(3)) cos(state(3)) state(2); 0 0 1];
    robot_pos = T * vehicle_points;
    
    % Plot chassis
    plot(robot_pos(1, :), robot_pos(2, :), 'b-', 'MarkerSize', 10);
    hold on;
    
    % Plot wheel directions
    quiver(robot_pos(1, 1), robot_pos(2, 1), cos(state(3)+psi(1)), sin(state(3)+psi(1)), 0.5, 'color',[1 0 0]);
    quiver(robot_pos(1, 2), robot_pos(2, 2), cos(state(3)+psi(2)), sin(state(3)+psi(2)), 0.5, 'color',[0 1 0]);
    quiver(robot_pos(1, 4), robot_pos(2, 4), cos(state(3)+psi(3)), sin(state(3)+psi(3)), 0.5, 'color',[0 0 1]);
    quiver(robot_pos(1, 3), robot_pos(2, 3), cos(state(3)+psi(4)), sin(state(3)+psi(4)), 0.5, 'color',[1 0 1]);
    
    % Plot wheel lateral
    %quiver(robot_pos(1, 1), robot_pos(2, 1), cos(state(3)+psi(1)+pi/2), sin(state(3)+psi(1)+pi/2), 2.5, 'color',[0 0 0]);
    %quiver(robot_pos(1, 2), robot_pos(2, 2), cos(state(3)+psi(2)+pi/2), sin(state(3)+psi(2)+pi/2), 2.5, 'color',[0 0 0]);
    %quiver(robot_pos(1, 4), robot_pos(2, 4), cos(state(3)+psi(3)+pi/2), sin(state(3)+psi(3)+pi/2), 2.5, 'color',[0 0 0]);
    %quiver(robot_pos(1, 3), robot_pos(2, 3), cos(state(3)+psi(4)+pi/2), sin(state(3)+psi(4)+pi/2), 2.5, 'color',[0 0 0]);
    
end

