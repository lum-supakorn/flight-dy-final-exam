% COVID-19 Aircraft Data

% Operating condition
altitude = 12500; % {ft} Flight altitude
M = 0.6; % {-} Mach number
a = 325.342; % {m/s} Speed of sound at the flight altitude
V0 = M * a; % {m/s} Freestream velocity
rho = 0.835679; % {kg/m^3} Far-field air density
q = 0.5 * rho * V0; % {Pa} Dynamic pressure

% Flight parameters
path_angle = 0; % {deg} Flight path angle
body_incidence = 6.3; % {deg} Body incidence

% Aircraft geometry
S = 330.980; % {m^2} Wing area
b = 51.787; % {m} Wingspan
mac = S / b; % {m} Mean aerodynamic chord

% Inertial parameters
m = 212100; % {kg} Mass
I_y = 514222; % {kgm^2} Pitch moment of inertia
I_x = 93210; % {kgm^2} Roll moment of inertia
I_z = 591514; % {kgm^2} Yaw moment of inertia
I_xz = 31008; % {kgm^2} Inertia product of roll and yaw moment

% Dimensionless stability derivatives
X_u = 0.0182;
X_w = 0.0778;
X_wdot = 0;
X_q = 0;
X_eta = 0.1786;

Z_u = -1.0251;
Z_w = -5.2421;
Z_wdot = -0.5101;
Z_q = -2.4546;
Z_eta = -0.9682;

M_u = 0.1249;
M_w = -0.8211;
M_wdot = -0.7690;
M_q = -1.654;
M_eta = -0.8212;

Y_v = -0.8717;
Y_p = 0;
Y_r = 0;
Y_xi = -0.0461;
Y_zeta = 0.1984;

L_v = -0.2282;
L_p = -0.2053;
L_r = 0.0810;
L_xi = 0.1780;
L_zeta = 0.1088;

N_v = 0.0489;
N_p = -0.0057;
N_r = -0.1623;
N_xi = 0.0024;
N_zeta = -0.0821;

% Save workspace to file
save('data/data.mat');