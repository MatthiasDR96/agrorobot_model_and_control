%% COntrol params

% Step size 
dt = 0.05;

% Maximum vehicle velocity
v_max = 1.0;

% End criterium
epsilon = 0.1;

% Signal-to-noise ratio for Gaussian noise on measurments
snr = 50;

%% Basic vehicle params

% Wheel diameter
r = 0.7;

% Massa robot
m = 650;
    
% Distances from center of gravity to wheel centers
CW_1_X = 0.8;
CW_2_X = 0.8;
CW_3_X = -0.8;
CW_4_X = -0.8;
CW_1_Y = 0.5;
CW_2_Y = -0.5;
CW_3_Y = 0.5;
CW_4_Y = -0.5;

% Steer limits
psi_min = -pi/4;
psi_max = pi/4;

% Inertias
I = 500; % Vehicle inertia about z-axis
Ia = 0.022; % Motor inertia
Is = 0.25; % Wheel inertia
It = 0.25; % Strut inertia
    
%% Wheel params

% Slip threshold
v_th = 0.1;

% Pacejka tire model params
% Dry tarmac: B=10, C=1.9, D=1, E=0.97
% Wet tarmac: B=12, C=2.3, D=0.82, E=1
B = 12;
C = 2.3;
D = 0.82;
E = 1;

% Lateral tire stiffness [N/rad]  
Cy = 1;

% Longitudinal tire stiffness [N]
Cx = 150000;

% Friction torque while steering
Es = 1;

% Rolling resistance coefficient
mu_0 = 0.16;

%% Motor params

% Gear ratio
gr = 50;

% Control params driving motors
J1 = 0.022 ;
Ts1 =1e-3;
Teq_i1=sqrt(2)*2*Ts1;
Kn=J1/2/Teq_i1;
Tn=20*Teq_i1;

% Control params steering motors
J2 = 0.022;
Ts2 = 1e-3;
Teq_i2 = sqrt(2)*2*Ts2;
Kn1 = J2/2/Teq_i2;
Tn1 = 20*Teq_i2;
Kn2 = J2/2/Teq_i2;
Tn2 = 20*Teq_i2;

% Torque threshold
T_thresh = 10;
T_thresh_s = 10;

% Motor saturations
fi_sat_min = -50;
fi_sat_max = 50;
psi_sat_min = -50;
psi_sat_max = 50;




