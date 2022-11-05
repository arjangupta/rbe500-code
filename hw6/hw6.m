% Code for HW6 of RBE500

clear; close all; clc;

% System model
J = 2;
B = 0.5;

% Controller
K_p = 11.55;
K_d = 8;
K_i = 0.6;

% Disturbance
D = 0.5;