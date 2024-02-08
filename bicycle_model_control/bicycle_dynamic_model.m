function [dx] = bicycle_dynamic_model(x, u)
    
    % Params
    m = 360;
    Iz = 100;
    Cr = 6;
    Cf = 6;
    lr = 0.5;
    lf = 0.5;

    % Lateral model
    V = x(2);
    A = [0 1 0 0; 0 -2*(Cr+Cf)/m*V 0 -((Cf*lf-Cr*lr)/(m*V))-V;...
        0 0 0 1; 0 -2*(Cf*lf-Cr*lr)/(Iz*V) 0 -2*(Cr*lr^2+Cf*lf^2)/(Iz*V)];
    B = [0; 2*Cf/m; 0; 2*Cf*lf/Iz];
    
    % Compute state derivative
    x_lateral = [x(3); x(4); x(5); x(6)];
    dx_lateral = A*x_lateral + B * u(2);
    
    % Dynamic model
    dx = zeros(6, 1);
    dx(1) = x(2);
    dx(2) = x(6) * x(4) + u(1);
    dx(3) = dx_lateral(1);
    dx(4) = dx_lateral(2);
    dx(5) = dx_lateral(3);
    dx(6) = dx_lateral(4);
    
end