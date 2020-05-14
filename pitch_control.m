% Pitch control
% Pitch damper

% Open-loop step response
t = 0:0.01:200;
step(q_eta * deg2rad(1), t);
hold on;
ylabel('Pitch angle {rad}');
title('Open-loop step response');

% Closed-loop step response
sys_cl = feedback(q_eta * deg2rad(1), -7.2);
step(sys_cl, t);
ylabel('Pitch angle {rad}');
title('Closed-loop step response');
% 
% % Goal
% % Overshoot less than 10%
% % Rise time less than 2 seconds
% % Settling time less than 10 seconds
% % Steady-state error less than 2%