function [fi_dot_req, psi_req] = inverse_kinematics(v, omega)

    % "System Design, Modelling, and Control of a
    % Four-Wheel-Steering Mobile Robot", from M. Makatchev et. al. 
    
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
   
    % Inverse kinematics
    if v==0 && omega==0
        
        % Wheel velocities
        fi_dot_1 = 0;
        fi_dot_2 = 0;
        fi_dot_3 = 0;
        fi_dot_4 = 0;
        
        % Wheel angles
        psi_1 = 0;
        psi_2 = 0;
        psi_3 = 0;
        psi_4 = 0;
        
    else
    
        % Wheel velocities
        fi_dot_1 = sign(v - omega * CW_1_Y) * sqrt((v - omega * CW_1_Y)^2 + (omega * CW_1_X)^2) / r;
        fi_dot_2 = sign(v - omega * CW_2_Y) * sqrt((v - omega * CW_2_Y)^2 + (omega * CW_2_X)^2) / r;
        fi_dot_3 = sign(v - omega * CW_3_Y) * sqrt((v - omega * CW_3_Y)^2 + (omega * CW_3_X)^2) / r;
        fi_dot_4 = sign(v - omega * CW_4_Y) * sqrt((v - omega * CW_4_Y)^2 + (omega * CW_4_X)^2) / r;

        % Wheel angles
        psi_1 = atan((omega * CW_1_X) / (v - omega * CW_1_Y));
        psi_2 = atan((omega * CW_2_X) / (v - omega * CW_2_Y));
        psi_3 = atan((omega * CW_3_X) / (v - omega * CW_3_Y));
        psi_4 = atan((omega * CW_4_X) / (v - omega * CW_4_Y));
        
    end
    
    % Output
    fi_dot_req = [fi_dot_1 fi_dot_2 fi_dot_3 fi_dot_4];
    psi_req = [psi_1 psi_2 psi_3 psi_4];
    
end

