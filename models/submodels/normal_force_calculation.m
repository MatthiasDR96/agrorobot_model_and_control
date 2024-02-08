function [normal_force] = normal_force_calculation()

    % Load params
    p = load('params.mat');
    m = p.m;
    CW_1_X = p.CW_1_X;
    CW_2_X = p.CW_2_X;
    CW_3_X = p.CW_3_X;
    CW_4_X = p.CW_4_X;
    
    % Normal forces due to load balance
    normal_force_1 = m * 9.81 * 0.5 * (abs(CW_3_X) / (CW_1_X + abs(CW_3_X)));
    normal_force_2 = m * 9.81 * 0.5 * (abs(CW_4_X) / (CW_2_X + abs(CW_4_X)));
    normal_force_3 = m * 9.81 * 0.5 * (CW_1_X / (CW_1_X + abs(CW_3_X)));
    normal_force_4 = m * 9.81 * 0.5 * (CW_2_X / (CW_2_X + abs(CW_4_X)));
    
    % Output
    normal_force = [normal_force_1, normal_force_2, normal_force_3, normal_force_4];
    
end

