% Longitudinal dynamics

clear
clc

load('long.mat');

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
% Response to transfer function
u_eta = tf(num(1, :), den);
w_eta = tf(num(2, :), den);
q_eta = tf(num(3, :), den);
theta_eta = tf(num(4, :), den);
alpha_eta = tf(num(5, :), den);
gamma_eta = tf(num(6, :), den);


% Find transfer function
G = ss(A,B,C,D);
tf(G)

% Determine steady step response
% Note: Steady state values can be found from the graph's characteristic
G1 = tf(u_eta * 0.01745)
step(G1, 100); % with specified time period (T_final=100)
title('$u(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);
step(G1); % without specified time period (until stable)
title('$u(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);

G2 = tf(w_eta * 0.01745)
step(G2, 100); % with specified time period (T_final=100)
title('$w(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);
step(G2); % without specified time period (until stable)
title('$w(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);

G3 = tf(q_eta * 0.01745)
step(G3, 100); % with specified time period (T_final=100)
title('$q(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);
step(G3); % without specified time period (until stable)
title('$q(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);

G4 = tf(theta_eta * 0.01745)
step(G4, 100); % with specified time period (T_final=100)
title('$\theta(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);
step(G4); % without specified time period (until stable)
title('$\theta(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);

G5 = tf(alpha_eta * 0.01745)
step(G5, 100); % with specified time period (T_final=100)
title('$\alpha(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);
step(G5); % without specified time period (until stable)
title('$\alpha(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);

G6 = tf(gamma_eta * 0.01745)
step(G6, 100); % with specified time period (T_final=100)
title('$\gamma(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);
step(G6); % without specified time period (until stable)
title('$\gamma(s) / \eta(s)$', 'Interpreter',"latex", "FontSize", 13);

% Factor denominator - Characteristic Eqn.
zpk(u_eta) % consider only the denominator

% Find parameters
% Phugoid
omega_p = sqrt(0.01114) 
zeta_p = 0.02186 / (2 * omega_p)
% short period
omega_s = sqrt(55.92)
zeta_s = (5.807) / (2 * omega_s)

% Find poles
pole(u_eta) 

% Eigenvector calculation
[e_vec, e_val] = eig(A);
abs(e_vec)

% Final Value Theorem to find steady state response
syms s
for i= 1:size(C,1)
    tr_f = poly2sym(num(i, :), sym('s')) / ... 
        poly2sym(den, sym('s'));
    final_val = vpa(limit(tr_f * s * (0.01745 / s)),5)
end

% Bode Diagram
bode(u_eta, {0, 10})
grid on
title("Axial Velocity Frequency Response", "FontSize", 13)

bode(w_eta, {0, 10})
grid on
title("Normal Velocity Frequency Response", "FontSize", 13)

bode(q_eta, {0, 10})
grid on
title("Pitch Rate Frequency Response", "FontSize", 13)

bode(theta_eta, {0, 10})
grid on
title("Pitch Frequency Response", "FontSize", 13)

bode(alpha_eta, {0, 10})
grid on
title("Angle of Attack Frequency Response", "FontSize", 13)

bode(gamma_eta, {0, 10})
grid on
title("Flight Path Angle Frequency Response", "FontSize", 13)

% Determine the bandwidth frequency of each response
bandwidth(u_eta)
bandwidth(w_eta)
bandwidth(q_eta)
bandwidth(theta_eta)
bandwidth(alpha_eta)
bandwidth(gamma_eta)