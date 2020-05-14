% Joshson book
% Example 4.4-1
% F-16 model

clear
clc

A = [
-1.9311E-02 8.8157E+00 -3.2170E+01 -5.7499E-01;
-2.5389E-04 -1.0189E+00 0.0000E+00 9.0506E-01;
0.0000E+00 0.0000E+00 0.0000E+00 1.0000E+00;
2.9465E-12 8.2225E-01 0.0000E+00 -1.0774E+00;
];

B = [
1.7370E-01;
-2.1499E-03;
0.0000E+00;
-1.7555E-01;
];

C = [
0.000000E+00 5.729578E+01 0.000000E+00 0.000000E+00;
0.000000E+00 0.000000E+00 0.000000E+00 5.729578E+01;
];

D = zeros(size(B, 2), size(C, 1))';

% State-space construction
[num, den] = ss2tf(A, B, C, D);

alpha_eta = tf(num(1, :), den);

% Augmented
A_aug = [
    A -B [0 0 0 0]';
    0 0 0 0 -20.2 0;
    0 10.0 0 0 0 -10.0;
];

B_aug = [0 0 0 0 20.2 0]';

C_aug = [
    C [0 0; 0 0];
    0 0 0 0 0 57.29578;
];

D_aug = zeros(size(B_aug, 2), size(C_aug, 1))';

% Root locus
k = logspace(-2, 1, 2000);
r = rlocus(A_aug, B_aug, C_aug(3,:), 0, k); % 3rd row of C
plot(r)
grid on
axis([-20,1,-10,10])

% State-space construction
[num_aug, den_aug] = ss2tf(A_aug, B_aug, C_aug(3,:), 0);

alpha_eta_aug = tf(num_aug, den_aug);