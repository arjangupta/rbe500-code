% Calculation code for problem 2 of the RBE500 Midterm

clear; close all; clc;

P2 = [2;5;0;1];

R2_1 = [1 0 0; 0 0.9553 0.2955; 0 -0.2955 0.9553];
d2_1 = [-1; -0.9553; 0.2955];
H_inv = [R2_1' (-R2_1'*d2_1); zeros(1,3) 1];

P1 = H_inv*P2;

p1 = P1(1:3,:)