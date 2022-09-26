clear; close all; clc;

syms theta1 d2 d3;

alpha = 0;
a = 0;
theta = theta1;
d = 1;

DH_3_10 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); ...
           sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); ...
           0 sin(alpha) cos(alpha) d; ...
           0 0 0 1];

disp(DH_3_10)

latex(DH_3_10)