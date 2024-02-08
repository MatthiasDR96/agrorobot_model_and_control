function Fy  = lateral_force_calculation(slip_angles)
    
    % "Longitudinal vehicle dynamics using Simulink/Matlab" from Payman
    % Shakouri et. al.
  
    % Load params
    p = load('params.mat');
    Cy = p.Cy;
    % D = p.D;
    % C = p.C;
    % B = p.B;
    % E = p.E;
    
    % Calculate friction coefficient with Pacejka tire model Short et. al. 2004
    % fric_coef_1 = D*sin(C*atan(B*slip_angle(1)-E*(B*slip_angle(1)-atan(B*slip_angle(1)))));
    % fric_coef_2 = D*sin(C*atan(B*slip_angle(2)-E*(B*slip_angle(2)-atan(B*slip_angle(2)))));
    % fric_coef_3 = D*sin(C*atan(B*slip_angle(3)-E*(B*slip_angle(3)-atan(B*slip_angle(3)))));
    % fric_coef_4 = D*sin(C*atan(B*slip_angle(4)-E*(B*slip_angle(4)-atan(B*slip_angle(4)))));
   
    % Normal force
    % normal_force = normal_force_calculation();
    
    % Calculate lateral force with Pacejka
    % Fy1 = fric_coef_1 * normal_force(1);
    % Fy2 = fric_coef_2 * normal_force(2);
    % Fy3 = fric_coef_3 * normal_force(3);
    % Fy4 = fric_coef_4 * normal_force(4);
    % Fy = [Fy1, Fy2, Fy3, Fy4]';
    
    % Calculate lateral force with linear approximation
    Fy = Cy * slip_angles; 
    
end

