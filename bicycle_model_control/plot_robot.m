function [] = plot_robot(state)
    
    % Vehicle coordinates
    vehicle_points = [0.5 0 1; -0.5 0 1]';

    % Plot center
    plot(state(1), state(2), 'b.', 'MarkerSize', 10);
    
    % Convert robot point to new pos
    T = [cos(state(3)) -sin(state(3)) state(1); sin(state(3)) cos(state(3)) state(2); 0 0 1];
    robot_pos = T * vehicle_points;
    
    % Plot chassis
    plot(robot_pos(1, :), robot_pos(2, :), '.b-', 'MarkerSize', 10);
    
    % Plot wheel directions
    %quiver(robot_pos(1, 1), robot_pos(2, 1), cos(state(3)+state(4)), sin(state(3)+state(4)), 0.1, 'color',[1 0 0]);
    %quiver(robot_pos(1, 2), robot_pos(2, 2), cos(state(3)), sin(state(3)), 0.1, 'color',[0 1 0]);
    
end

