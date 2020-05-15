clear
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
% Longitudinal state space construction
% Referred to the body-fixed axis!



% load('data/data.mat');

% Other important derived parameters
m_prime = m / (q * S);
I_y_prime = I_y / (q * S * mac);
U_e = V0 * cosd(body_incidence);
W_e = V0 * sind(body_incidence);
g = 9.81;

% Inverse matrix and vector construction
M = ...
[
    m_prime, -(X_wdot * mac)/V0, 0, 0;
    0, (m_prime - ((Z_wdot * mac)/V0)), 0, 0;
    0, -((M_wdot * mac)/V0), I_y_prime, 0;
    0, 0, 0, 1
];

A_prime = ...
[
	X_u, X_w, ((X_q * mac) - (m_prime * W_e)), -(m_prime * g * cosd(body_incidence));
    Z_u, Z_w, ((Z_q * mac) + (m_prime * U_e)), -(m_prime * g * sind(body_incidence));
    M_u, M_w, M_q * mac, 0;
    0, 0, 1, 0;
];

B_prime = [V0 * X_eta, V0 * Z_eta, V0 * M_eta, 0]';

% Concise state space form
A = M\A_prime;
B = M\B_prime;

% disp(A)
% disp(B)
% disp(A_r)
% disp(B_r)

% Save the state matrix and input vector
% save('data/long.mat', 'A', 'B', 'V0');
% Output matrix
C = ...
[
    1, 0, 0, 0;
    0, 1, 0 ,0;
    0, 0, 1, 0;
    0, 0, 0, 1;
    0, 1/V0, 0, 0;
    0, -1/V0, 0, 0;
];

% Feedthrough vector
D = zeros(size(B, 2), size(C, 1))';

% State-space construction
[num, den] = ss2tf(A, B, C, D);

% All parameters are per radian
u_eta = tf(num(1, :)*0.01745, den);
w_eta = tf(num(2, :)*0.01745, den);
q_eta = tf(num(3, :)*0.01745, den);
theta_eta = tf(num(4, :), den);
alpha_eta = tf(num(5, :), den);
gamma_eta = tf(num(6, :), den);

% Eigenvector of [A]
%[V,D] = eig(A);
% display(V)
% % Magnitude 
% abs(V)

% Find the Steady State Values
k_u = dcgain(u_eta)*0.01745
k_w = dcgain(w_eta)*0.01745
k_q = dcgain(q_eta)*0.01745
k_theta = dcgain(theta_eta)
k_alpha = dcgain(alpha_eta)
k_gamma = dcgain(gamma_eta)

% step(gamma_eta)

% Reduce Order Model
A_r = zeros(2);
B_r = zeros(2,1);

A_r(1,1) = A(2,2);
A_r(1,2) = A(2,3);
A_r(2,1) = A(3,2);
A_r(2,2) = A(3,3);

B_r(1,1) = B(2,1);
B_r(2,1) = B(3,1);

C_r = [1 0;
       0 1;
       1/V0 0];
D_r = [0;0;0];

[num_r,den_r]=ss2tf(A_r,B_r,C_r,D_r);

w_eta_r = tf(num_r(1, :)*0.01745, den_r);
q_eta_r = tf(num_r(2, :)*0.01745, den_r);
theta_eta_r = tf(num_r(3, :), den_r);

% step(w_eta)
% title('$w(s)/\eta(s)$', 'Interpreter',"latex", "FontSize", 13)
% hold on
% step(w_eta_r)
% legend({'Full order Model','Reduced order Model)'},'Location','southeast')
% 
% step(q_eta)
% title('$q(s)/\eta(s)$', 'Interpreter',"latex", "FontSize", 13)
% hold on
% step(q_eta_r)
% legend({'Full order Model','Reduced order Model)'},'Location','southeast')

% step(theta_eta)
% title('$\theta(s)/\eta(s)$', 'Interpreter',"latex", "FontSize", 13)
% hold on
% step(theta_eta_r)
% legend({'Full order Model','Reduced order Model'},'Location','northeast')
