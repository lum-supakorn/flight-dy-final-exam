% Lateral-directional dynamics

clear
clc

load('lat_dir.mat');

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

% Find transfer function
G = ss(A,B,C,D);
tf(G)

% Graph transfer function
% Pulse input for Aileron
% t = 0:0.01:30;
% t2 = 0:0.01:400;
% u = (heaviside(t) - heaviside(t - 2)) * 0.01745;
% u2 = (heaviside(t2) - heaviside(t2 - 2)) * 0.01745;
% lsim(v_xi, u, t)
% title('$v(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% lsim(v_xi, u2, t2)
% title('$v(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% 
% lsim(p_xi, u, t)
% title('$p(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% lsim(p_xi, u2, t2)
% title('$p(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% 
% lsim(r_xi, u, t)
% title('$r(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% lsim(r_xi, u2, t2)
% title('$r(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% 
% lsim(phi_xi, u, t)
% title('$\phi(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% lsim(phi_xi, u2, t2)
% title('$\phi(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% 
% lsim(psi_xi, u, t)
% title('$\psi(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% lsim(psi_xi, u2, t2)
% title('$\psi(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% 
% lsim(beta_xi, u, t)
% title('$\beta(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)
% lsim(beta_xi, u2, t2)
% title('$\beta(s) / \xi(s)$', 'Interpreter',"latex", "FontSize", 13)

% Step input for Rudder
G1_r = tf(v_zeta * 0.01745)
step(G1_r, 30) % with specified time period (T_final=30)
title('$v(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)
step(G1_r) % without specified time period (until stable)
title('$v(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)

G2_r = tf(p_zeta * 0.01745)
step(G2_r, 30)
title('$p(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)
step(G2_r)
title('$p(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)

G3_r = tf(r_zeta * 0.01745)
step(G3_r, 30)
title('$r(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)
step(G3_r)
title('$r(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)

G4_r = tf(phi_zeta * 0.01745)
step(G4_r, 30)
title('$\phi(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)
step(G4_r)
title('$\phi(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)

G5_r = tf(psi_zeta * 0.01745)
step(G5_r, 30)
title('$\psi(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)
step(G5_r)
title('$\psi(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)
 
G6_r = tf(beta_zeta * 0.01745)
step(G6_r, 30)
title('$\beta(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)
step(G6_r)
title('$\beta(s) / \zeta(s)$', 'Interpreter',"latex", "FontSize", 13)

% Factor denominator - Characteristic Eqn.
zpk(v_xi)
zpk(v_zeta) % verify

% Find poles
pole(v_xi) 
pole(v_zeta) % verify

% Eigenvector calculation
[e_vec, e_val] = eig(A);
abs(e_vec)

% Final Value Theorem to find steady state response
syms s
% Aileron
for i= 1:size(C,1)
    tr_f = poly2sym(num_xi(i, :), sym('s')) / ... 
        poly2sym(den, sym('s'));
    final_val = vpa(limit(tr_f * s * (0.01745 / s)),5)
end

% Rudder
for i= 1:size(C,1)
    tr_f = poly2sym(num_zeta(i, :), sym('s')) / ... 
        poly2sym(den, sym('s'));
    final_val = vpa(limit(tr_f * s * (0.01745 / s)),5)
end

% Bode diagram
bode(v_xi, {0, 10})
grid on
title("Lateral Velocity Frequency Response", "FontSize", 13)

bode(p_xi, {0, 10})
grid on
title("Roll Rate Frequency Response", "FontSize", 13)

bode(r_xi, {0, 10})
grid on
title("Yaw Rate Frequency Response", "FontSize", 13)

bode(phi_xi, {0, 10})
grid on
title("Roll Angle Frequency Response", "FontSize", 13)

bode(psi_xi, {0, 10})
grid on
title("Yaw Angle Frequency Response", "FontSize", 13)

bode(beta_xi, {0, 10})
grid on
title("Sideslip Angle Frequency Response", "FontSize", 13)

bode(v_zeta, {0, 10})
grid on
title("Lateral Velocity Frequency Response", "FontSize", 13)

bode(p_zeta, {0, 10})
grid on
title("Roll Rate Frequency Response", "FontSize", 13)

bode(r_zeta, {0, 10})
grid on
title("Yaw Rate Frequency Response", "FontSize", 13)

bode(phi_zeta, {0, 10})
grid on
title("Roll Angle Frequency Response", "FontSize", 13)

bode(psi_zeta, {0, 10})
grid on
title("Yaw Angle Frequency Response", "FontSize", 13)

bode(beta_zeta, {0, 10})
grid on
title("Sideslip Angle Frequency Response", "FontSize", 13)

% Determine the bandwidth frequency of each response
bandwidth(v_xi)
bandwidth(p_xi)
bandwidth(r_xi)
bandwidth(phi_xi)
bandwidth(psi_xi)
bandwidth(beta_xi)

bandwidth(v_zeta)
bandwidth(p_zeta)
bandwidth(r_zeta)
bandwidth(phi_zeta)
bandwidth(psi_zeta)
bandwidth(beta_zeta)