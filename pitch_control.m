% Pitch control

clear
clc

load('data/long_tf.mat');

ele_input = deg2rad(11);

% Open-loop step response
t = 0:0.01:400;
step(ele_input * alpha_eta, t);
ylabel('Pitch angle {rad}');
title('Open-loop step response');

% Closed-loop step response
sys_cl = feedback(alpha_eta, 1);
step(ele_input * sys_cl, t);
ylabel('Pitch angle {rad}');
title('Closed-loop step response');