% Calculation code for problem 4.15 (Jacobian) of the RBE500 textbook (HW 4)

clear; close all; clc;

syms d1 d2 d3;

% Declare vectors
z0 = [0; 0; 1];
z1 = z0;
z2 = z0;
o0 = [0; 0; 0];
o3 = [0; 0; d1 + d2 + d3];

% Compute Jacobian
J = [cross(z0, (o3 - o0)) z1 z2; z0 zeros(3,1) zeros(3,1)]

% Output LaTex
latex(J)