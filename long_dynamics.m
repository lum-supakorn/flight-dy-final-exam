% Longitudinal dynamics

clear
clc

load('data/long.mat');

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
u_eta = tf(num(1, :), den);
w_eta = tf(num(2, :), den);
q_eta = tf(num(3, :), den);
theta_eta = tf(num(4, :), den);
alpha_eta = tf(num(5, :), den);
gamma_eta = tf(num(6, :), den);

save('data/long_tf.mat', 'alpha_eta');