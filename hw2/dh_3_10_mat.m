clear; close all; clc;

syms theta1 d1 d2 d3;

alpha = -pi/2;
a = 0;
theta = theta1;
d = d1;

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