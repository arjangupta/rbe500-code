% Calculation code for problem 2 of the RBE500 Midterm

clear; close all; clc;

P2 = [2;5;0;1];
H2_1 = [1 0 0 -1; 0 0.9553 0.2955 -0.9553; 0 -0.2955 0.9553 0.2955; 0 0 0 1];
inv(H2_1)

% Generate LaTex code
% latex(T)