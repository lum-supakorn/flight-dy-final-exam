% 2-s pulse input for ailerons

t = 0:0.1:100;
u = (heaviside(t) - heaviside(t-2)) * deg2rad(1);
lsim(v_xi, u, t)