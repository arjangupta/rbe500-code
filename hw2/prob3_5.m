% Calculation code for problem 3.5 of the RBE500 textbook (HW 2)

clear; close all; clc;

syms c1 s1 d1 c2 s2 a2 c3 s3 a3;
A1 = [c1 0 -s1 0; s1 0 c1 0; 0 -1 0 d1; 0 0 0 1];
A2 = [c2 -s2 0 a2*c2; s2 c2 0 a2*s2; 0 0 1 0; 0 0 0 1];
A3 = [c3 -s3 0 a3*c3; s3 c3 0 a3*s3; 0 0 1 0; 0 0 0 1];

T = A1*A2*A3;

% Generate LaTex code
latex(T)