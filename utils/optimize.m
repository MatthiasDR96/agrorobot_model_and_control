function [x] = optimize(model, control, traj, max_sim_time, noise, render, nvars, lb, ub)

    % GA solver
    options = optimoptions('ga','ConstraintTolerance',1e-6,'PlotFcn', ...
        {@gaplotbestf,@gaplotstopping}, 'MaxGenerations', 20, 'MaxStallGenerations', ...
        5, 'PopulationSize', 10);
    x = ga(@(x) obj_fun(x, model, control, traj, max_sim_time, noise, render), ...
        nvars,[],[],[],[],lb,ub,[],[],options);

end

function [error] = obj_fun(x, model, control, traj, max_sim_time, noise, render)

    % Simulate
    [~, x_hist, traj] = simulate(model, control, traj, max_sim_time, noise, render, x);
    
    % Fitness
    x_err = (traj(1,1) - x_hist(1, :)).^2;
    y_err = (traj(2,1) - x_hist(2, :)).^2;
    error = sum(sqrt(x_err + y_err));

end