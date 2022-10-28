% Calculation code for step 3 of Question 5 of the RBE500 midterm

clear; close all; clc;

syms d1 d2 d3;
A1 = [1 0 0 0; 0 0 -1 0; 0 1 0 d1; 0 0 0 1];
A2 = [0 0 -1 0; -1 0 0 0; 0 1 0 d2; 0 0 0 1];
A3 = [1 0 0 0; 0 1 0 0; 0 0 1 d3; 0 0 0 1];

T = A1*A2*A3;

% Generate LaTex code
latex(T)