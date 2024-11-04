clear all;
close all;
clc;

syms m g t lambda radius r(t) r_dot(t) r_ddot(t) theta(t) theta_dot(t) theta_ddot(t) phi(t) phi_dot(t) phi_ddot(t) real;


r_dot(t) = diff(r, t);
theta_dot(t) = diff(theta, t);
phi_dot(t) = diff(phi, t);

r_ddot(t) = diff(r_dot, t);
theta_ddot(t) = diff(theta_dot, t);
phi_ddot(t) = diff(phi_dot, t);

q = [r;theta;phi];
q_dot = [r_dot;theta_dot;phi_dot];
q_ddot = [r_ddot;theta_ddot;phi_ddot];

lagrangian = 0.5 * m * (r_dot^2 + r^2 * phi_dot^2 + r^2*sin(phi * theta_dot^2)^2) + m * g * r * cos(phi) + lambda * r^2;

 % r(t) = 1;


syms radius th ph dradius dth dph ddradius ddth ddph real; 

eq_motion = diff(gradient(lagrangian, q_dot), t) - gradient(lagrangian, q);

eq_motion = formula(eq_motion);
eq_motion = subs(eq_motion, [r], [radius]);

eq_motion = subs(eq_motion, [q_ddot], [ddradius ddth ddph]');
eq_motion = subs(eq_motion, [q_dot], [dradius dth dph]');
eq_motion = subs(eq_motion, [theta;phi], [th ph]');

eq_motion = simplify(eq_motion);
% eq_motion = subs(eq_motion, diff(diff(q,t), t), q_dot);
% eq_motion = subs(eq_motion, diff(q, t), q_dot);


dynamics_sol = solve(eq_motion == 0,[th ph]);