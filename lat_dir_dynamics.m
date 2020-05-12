% Lateral-directional dynamics

clear
clc

load('data/lat_dir.mat');

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

% All parameters are per radian

% Aileron
v_xi = tf(num_xi(1, :), den);
p_xi = tf(num_xi(2, :), den);
r_xi = tf(num_xi(3, :), den);
phi_xi = tf(num_xi(4, :), den);
psi_xi = tf(num_xi(5, :), den);
beta_xi = tf(num_xi(6, :), den);

% Rudder
v_zeta = tf(num_zeta(1, :), den);
p_zeta = tf(num_zeta(2, :), den);
r_zeta = tf(num_zeta(3, :), den);
phi_zeta = tf(num_zeta(4, :), den);
psi_zeta = tf(num_zeta(5, :), den);
beta_zeta = tf(num_zeta(6, :), den);