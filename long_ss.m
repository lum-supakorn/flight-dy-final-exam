% Longitudinal state space construction

clear
clc

load('data.mat');

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

% Save the state matrix and input vector
save('long.mat', 'A', 'B', 'V0');