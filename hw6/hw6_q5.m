% Code for HW6 of RBE500

clear; close all; clc;

% System model
J = 2;
B = 0.5;

% Controller
K_p = 11;
K_d = 8;
K_i = 20;

% Disturbance
D = 0.5;