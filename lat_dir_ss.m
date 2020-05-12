% Longitudinal state space construction
% Referred to the body-fixed axis!

clear
clc

load('data.mat');

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

% Save the state matrix and input vector
save('lat_dir.mat', 'A', 'B', 'V0');