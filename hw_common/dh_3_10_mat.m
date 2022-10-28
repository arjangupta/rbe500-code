clear; close all; clc;

syms theta1 theta2 theta4 theta5 theta6 a2 a3 d1 d2 d3 l4 l5 l6;

alpha = 0; % in radians
a = 0;
theta = theta5; % in radians
d = l6;

% DH_3_10 = [round(cos(theta),2) round(-sin(theta)*cos(alpha),2) sin(theta)*sin(alpha) a*cos(theta); ...
%            round(sin(theta),2) round(cos(theta)*cos(alpha),2) round(-cos(theta)*sin(alpha),2) a*sin(theta); ...
%            0 round(sin(alpha),2) round(cos(alpha), 2) d; ...
%            0 0 0 1];

DH_3_10 = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); ...
           sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); ...
           0 sin(alpha) cos(alpha) d; ...
           0 0 0 1];

disp(DH_3_10)

latex(DH_3_10)