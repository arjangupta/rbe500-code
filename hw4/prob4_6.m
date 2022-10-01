% Calculation code for problem 4.6 of the RBE500 textbook (HW 4)

clear; close all; clc;

syms theta phi

% Define the matrices in our problem
rotx_theta = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
Sj = [0 0 1; 0 0 0; -1 0 0];
roty_phi = [cos(phi) 0 sin(phi); 0 1 0; -sin(phi) 0 cos(phi)];

% Multiply the matrices
product = rotx_theta*Sj*roty_phi;

% Get latex output
latex(product)

phi_val = pi/2;
theta_val = pi/2;

% Now plug in values
product_val = [ -sin(phi_val), 0, cos(phi_val);
cos(phi_val)*sin(theta_val), 0, sin(phi_val)*sin(theta_val);
-cos(phi_val)*cos(theta_val), 0, -cos(theta_val)*sin(phi_val)]