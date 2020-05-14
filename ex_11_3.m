% Example 11.3
% Lockheed F-104 Starfighter

clear
clc

% Pitch rate response to elevator transfer function
% Transfer function construction

s = tf('s');
num = -4.66 * s * (s + 0.133) * (s + 0.269);
den = (s^2 + (0.015 * s) + 0.021) * (s^2 + (0.911 * s) + 4.884);

sys = num/den;

% Stability modes
damp(sys)

% ## Goals ##
% Phugoid damping ratio >= 0.04
% Short-period damping ratio >= 0.5
% Short-period undamped natural frequency >= 0.8 and <= 3.0 rad/s

t = 0:0.1:25;

step(sys, t);
hold on;

sys_cl = feedback(sys, -0.5);
step(sys_cl, t);

% Root locus
% k = -100:0.01:10;
% rlocus(sys, k);
