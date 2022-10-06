% Calculation code for problem 4.15 of the RBE500 textbook (HW 4)

clear; close all; clc;

syms theta1 d1 d2 d3;

% Form the A matrices
A1 = [cos(theta1) -sin(theta1) 0 0; sin(theta1) cos(theta1) 0 0; 0 0 1 d1; 0 0 0 1];
A2 = [1 0 0 0; 0 1 0 0; 0 0 1 d2; 0 0 0 1];
A3 = [1 0 0 0; 0 1 0 0; 0 0 1 d3; 0 0 0 1];

% Compute T matrices
T2 = A1*A2;
T3 = A1*A2*A3;

% Output to LaTex
latex(T2)
latex(T3)