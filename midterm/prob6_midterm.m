% Calculation code for Question 6 of the RBE500 midterm

clear; close all; clc;

% Declare symbolic variables/constants
syms d1 d2 d3 theta4 theta5 theta6 l4 l5 l6;

% Write out our A matrices
A1 = [1 0 0 0; 0 0 -1 0; 0 1 0 d1; 0 0 0 1];
A2 = [0 0 -1 0; -1 0 0 0; 0 1 0 d2; 0 0 0 1];
A3 = [1 0 0 0; 0 1 0 0; 0 0 1 d3; 0 0 0 1];
A4 = [cos(theta4) 0 sin(theta4) 0; sin(theta4) 0 -cos(theta4) 0; 0 1 0 l4; 0 0 0 1];
A5 = [cos(theta5) 0 -sin(theta5) 0; sin(theta5) 0 cos(theta5) 0; 0 -1 0 l5; 0 0 0 1];
A6 = [cos(theta6) -sin(theta6) 0 0; sin(theta6) cos(theta6) 0 0; 0 0 1 l6; 0 0 0 1];

% Compute T matrices
T2 = A1*A2;
T3 = T2*A3;
T4 = T3*A4;
T5 = T4*A5;
T6 = T5*A6;

R_base = eye(3);

% Extract required z vectors
z0 = R_base(:,3);
z1 = A1(1:3,3);
z2 = T2(1:3,3);
z3 = T3(1:3,3);
z4 = T4(1:3,3);
z5 = T5(1:3,3);

% Extract required o vectors
o3 = T3(1:3,4);
o4 = T4(1:3,4);
o5 = T5(1:3,4);
o6 = T6(1:3,4);

% Calculate the Jacobian matrix
J = [z0 z1 z2 cross(z3,(o6-o3)) cross(z4,(o6-o4)) cross(z5,(o6-o5)); zeros(3,1) zeros(3,1) zeros(3,1) z3 z4 z5];

% Export to LaTeX
latex(J)