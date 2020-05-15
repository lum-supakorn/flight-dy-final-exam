clear;
clc
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
% save('data/data.mat');

% load('data/data.mat');

% Other important derived parameters
m_prime = m / (q * S);
I_x_prime = I_x / (q * S * b);
I_z_prime = I_z / (q * S * b);
I_xz_prime = I_xz / (q * S * b);
U_e = V0 * cosd(body_incidence);
W_e = V0 * sind(body_incidence);
g = 9.81;

% Inverse matrix and vector construction
M = ...
[
    m_prime, 0, 0, 0, 0;
    0, I_x_prime, -I_xz_prime, 0, 0;
    0, -I_xz_prime, I_z_prime, 0, 0;
    0, 0, 0, 1, 0;
    0, 0, 0, 0, 1;
];

A_prime = ...
[
	Y_v, ((Y_p * b) + (m_prime * W_e)), ((Y_r * b) - (m_prime * U_e)), (m_prime * g * cosd(body_incidence)), (m_prime * g * sind(body_incidence));
    L_v, L_p * b, L_r * b, 0, 0;
    N_v, N_p * b, N_r * b, 0, 0;
    0, 1, 0, 0, 0;
    0, 0, 1, 0, 0;
];

B_prime = ...
[
    V0 * Y_xi, V0 * L_xi, V0 * N_xi, 0, 0;
    V0 * Y_zeta, V0 * L_zeta, V0 * N_zeta, 0, 0;
]';

% Concise state space form
A = M\A_prime;
B = M\B_prime;

% Output matrix
C = ...
[
    1, 0, 0, 0, 0;
    0, 1, 0 ,0, 0;
    0, 0, 1, 0, 0;
    0, 0, 0, 1, 0;
    0, 0, 0, 0, 1;
    1/V0, 0, 0, 0, 0;
];

% Feedthrough vector
D = zeros(size(B, 2), size(C, 1))';

% State-space construction
[num_xi, ~] = ss2tf(A, B, C, D, 1);
[num_zeta, den] = ss2tf(A, B, C, D, 2);
pole = roots(den);
% All parameters are per radian

% Aileron
v_xi = tf(num_xi(1, :)*0.01745, den);
p_xi = tf(num_xi(2, :)*0.01745, den);
r_xi = tf(num_xi(3, :), den);
phi_xi = tf(num_xi(4, :)*0.01745, den);
psi_xi = tf(num_xi(5, :), den);
beta_xi = tf(num_xi(6, :), den);

% Rudder
v_zeta = tf(num_zeta(1, :), den);
p_zeta = tf(num_zeta(2, :)*0.01745, den);
r_zeta = t
f(num_zeta(3, :), den);
phi_zeta = tf(num_zeta(4, :)*0.01745, den);
psi_zeta = tf(num_zeta(5, :), den);
beta_zeta = tf(num_zeta(6, :), den);

%Reduced Order Model

%Roll mode
A_roll = zeros(2);
A_roll(1,1) = A(2,2);
A_roll(1,2) = A(2,4);
A_roll(2,1) = 1;
A_roll(2,2) = 0;
B_roll = zeros(2);
B_roll(1,1) = B(2,1);
B_roll(1,2) = B(2,2);
B_roll(2,1) = 0;
B_roll(2,2) = 0;
C_roll = eye(2);
D_roll = zeros(2);

[num_roll_xi,~]=ss2tf(A_roll,B_roll,C_roll,D_roll,1);
[num_roll_zeta,den_roll]=ss2tf(A_roll,B_roll,C_roll,D_roll,2);

%p_xi_roll = tf(num_roll_xi(1, :)*0.01745, den_roll);
%phi_xi_roll = tf(num_roll_xi(2, :)*0.01745, den_roll);

p_zeta_roll = tf(num_roll_zeta(1, :)*0.01745, den_roll);
phi_zeta_roll = tf(num_roll_zeta(2, :)*0.01745, den_roll);


% Step Response
step(p_zeta)
% title('$\p(s)/\xi(s)$', 'Interpreter',"latex", "FontSize", 13)
hold on
step(p_zeta_roll)
legend({'Full order Model','Reduced order Model, Roll Mode'},'Location','southeast')

% sys_sim = tf(3.2,[9.5877 1],'InputDelay',2.241)
% step(v_xi)
% hold on
% step(sys_sim)

