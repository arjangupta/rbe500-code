% Calculation code for problem 2.37 of RBE500 Textbook

clc; clear; close all;

% Obtain rotation matrices
R_0_1 = rotx(90) * rotz(-90)
R_0_2 = rotz(-90) * rotx(90)
R_1_2 = rotz(90) * roty(-90)

% Distances for homogenous matrices
d_0_1 = [0 0 1]';
d_0_2 = [0 1 0]';
d_1_2 = [1 0 -1]';

% Form the homogeneous matrices
H_0_1 = [R_0_1 d_0_1; zeros(1,3) 1];
H_0_2 = [R_0_2 d_0_2; zeros(1,3) 1];
H_1_2 = [R_1_2 d_1_2; zeros(1,3) 1];

% Show the multiplication
product = H_0_1 * H_1_2

% Verify
if isequal(product, H_0_2)
    fprintf('Verified!\n');
end