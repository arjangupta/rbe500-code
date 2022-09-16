% Calculation code for problem 2.38 of the RBE500 textbook (HW 1)

clc; clear; close all;

R_0_3 = rotx(-180) * rotz(-90)
R_3_2 = roty(180) * rotz(90)

% Verification (not part of required problem)
if isequal(R_0_3', R_3_2)
    fprintf('Rotations verified!\n');
end