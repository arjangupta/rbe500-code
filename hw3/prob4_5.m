% Calculation code for problem 4.5 of the RBE500 textbook (HW 3)

clear; close all; clc;

rotx90 = rotx(90);
Sa = [0 -2 -1; 2 0 -1; 1 1 0];

step2 = rotx90*Sa;
step3 = step2*rotx90';

a = [1,-1,2]';
rotx90*a
