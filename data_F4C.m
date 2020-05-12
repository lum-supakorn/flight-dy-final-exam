% McDonnell F-4C Aircraft Data

% Operating condition
altitude = 12500; % {ft} Flight altitude
M = 0.6; % {-} Mach number
a = 325.342; % {m/s} Speed of sound at the flight altitude
V0 = 178; % {m/s} Freestream velocity
rho = 0.3809; % {kg/m^3} Far-field air density
q = 0.5 * rho * V0; % {Pa} Dynamic pressure

% Flight parameters
path_angle = 0; % {deg} Flight path angle
body_incidence = 9.4; % {deg} Body incidence

% Aircraft geometry
S = 49.239; % {m^2} Wing area
b = 11.787; % {m} Wingspan
mac = S / b; % {m} Mean aerodynamic chord
%mac = 4.889; % {m} Mean aerodynamic chord
%b = S / mac; % {m} Wingspan

% Inertial parameters
m = 17642; % {kg} Mass
I_y = 165669; % {kgm^2} Pitch moment of inertia
I_x = 33898; % {kgm^2} Roll moment of inertia
I_z = 189496; % {kgm^2} Yaw moment of inertia
I_xz = 2952; % {kgm^2} Inertia product of roll and yaw moment

% Dimensionless stability derivatives
X_u = 0.0076;
X_w = 0.0483;
X_wdot = 0;
X_q = 0;
X_eta = 0.0618;

Z_u = -0.7273;
Z_w = -3.1245;
Z_wdot = -0.3997;
Z_q = -1.2109;
Z_eta = -0.3741;

M_u = 0.0340;
M_w = -0.2169;
M_wdot = -0.5910;
M_q = -1.2732;
M_eta = -0.5581;

Y_v = -0.5974;
Y_p = 0;
Y_r = 0;
Y_xi = -0.0159;
Y_zeta = 0.1193;

L_v = -0.1048;
L_p = -0.1164;
L_r = 0.0455;
L_xi = 0.0454;
L_zeta = 0.0086;

N_v = 0.0987;
N_p = -0.0045;
N_r = -0.1132;
N_xi = 0.00084;
N_zeta = -0.0741;

% Save workspace to file
save('data/data_F4C.mat');