% Calculation code for step 4 of Question 5 of the RBE500 midterm
clear; close all; clc;

syms c1 s1 r11 r12 r13 r21 r22 r23 r31 r32 r33;

R30 = [0 0 -1; 0 -1 0; -1 0 0];
R06 = [r11 r12 r13; r21 r22 r23; r31 r32 r33];

R36 = R30*R06;

latex(R36)