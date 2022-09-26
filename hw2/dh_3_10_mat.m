clear; close all; clc;

syms d1 d2;

alpha = pi/2;
a = 0;
theta = 0;
d = d1;

DH_3_10 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); ...
           sin(theta) round(cos(theta)*cos(alpha),2) -cos(theta)*sin(alpha) a*sin(theta); ...
           0 sin(alpha) round(cos(alpha), 2) d; ...
           0 0 0 1];

disp(DH_3_10)