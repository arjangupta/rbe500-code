clear; close all; clc;

syms theta1 theta2 a2 d1 d2 d3;

alpha = 0;
a = a2;
theta = theta2;
d = 0;

% DH_3_10 = [round(cos(theta),2) round(-sin(theta)*cos(alpha),2) sin(theta)*sin(alpha) a*cos(theta); ...
%            sin(theta) round(cos(theta)*cos(alpha),2) round(-cos(theta)*sin(alpha),2) a*sin(theta); ...
%            0 sin(alpha) round(cos(alpha), 2) d; ...
%            0 0 0 1];

DH_3_10 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); ...
           sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); ...
           0 sin(alpha) cos(alpha) d; ...
           0 0 0 1];

disp(DH_3_10)

latex(DH_3_10)