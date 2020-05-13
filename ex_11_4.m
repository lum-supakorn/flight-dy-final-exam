% Example 11.4
% McDonnell Douglas A-4D Skyhawk

clear
clc

s = tf('s');

% Pitch attitude feedback to elevator
theta_eta = ...
-8.096 * (s - 0.0006) * (s + 0.3591) / ...
((s^2 + 0.014 * s + 0.0068) * (s^2 + 1.009 * s + 5.56));
