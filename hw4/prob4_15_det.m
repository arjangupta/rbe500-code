% Calculation code for problem 4.15 (Determinant) of the RBE500 textbook (HW 4)

clear; close all; clc;

% Declare symbolic variables
syms d3 theta1;

% Write J11 (first three rows of our jacobian)
J11 = [-d3*cos(theta1) 0 -sin(theta1); -d3*sin(theta1) 0 cos(theta1); 0 1 0];
% Compute the determinant
det_J11 = det(J11);

% Output to LaTex
latex(det_J11)