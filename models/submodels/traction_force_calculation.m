function Fx  = traction_force_calculation(slip_ratios)
    
    % "Longitudinal vehicle dynamics using Simulink/Matlab" from Payman
    % Shakouri et. al.
    
    % Load params
    p = load('params.mat');
    % Cx = p.Cx;
    D = p.D;
    C = p.C;
    B = p.B;
    E = p.E;
    
    % Calculate friction coefficient with Pacejka tire model Short et. al. 2004
    fric_coef_1 = D*sin(C*atan(B*slip_ratios(1)-E*(B*slip_ratios(1)-atan(B*slip_ratios(1)))));
    fric_coef_2 = D*sin(C*atan(B*slip_ratios(2)-E*(B*slip_ratios(2)-atan(B*slip_ratios(2)))));
    fric_coef_3 = D*sin(C*atan(B*slip_ratios(3)-E*(B*slip_ratios(3)-atan(B*slip_ratios(3)))));
    fric_coef_4 = D*sin(C*atan(B*slip_ratios(4)-E*(B*slip_ratios(4)-atan(B*slip_ratios(4)))));
   
    % Normal force
    normal_force = normal_force_calculation();
    
    % Calculate traction force with Pacejka
    Fx1 = fric_coef_1 * normal_force(1);
    Fx2 = fric_coef_2 * normal_force(2);
    Fx3 = fric_coef_3 * normal_force(3);
    Fx4 = fric_coef_4 * normal_force(4);
    
    % Output
    Fx = [Fx1, Fx2, Fx3, Fx4]';
    
    % Calculate traction force with constant approximation
    %Fx = Cx * slip_ratios;

end
