clear; close all; clc;

syms d1 d2 d3;

alpha = pi/2;
a = 0;
theta = pi/2;
d = d2;

DH_3_10 = [round(cos(theta),2) round(-sin(theta)*cos(alpha),2) sin(theta)*sin(alpha) a*cos(theta); ...
           sin(theta) round(cos(theta)*cos(alpha),2) round(-cos(theta)*sin(alpha),2) a*sin(theta); ...
           0 sin(alpha) round(cos(alpha), 2) d; ...
           0 0 0 1];

disp(DH_3_10)