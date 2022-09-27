% Calculation code for problem 4.3 of the RBE500 textbook (HW 3)

clear; close all; clc;

syms p q r s t u v w x a1 a2 a3 b1 b2 b3;

R = [p q r; s t u; v w x];
a = [a1; a2; a3];
b = [b1; b2; b3];

cross(R * a, R * b)
R * cross(a, b)