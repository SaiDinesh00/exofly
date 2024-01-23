clear all
clc
close all


% Quadcopter parameters
mass = 1.0; % kg
gravity = 9.81; % m/s^2
omega = [500; 500; 500; 500];
inertia = [1; 1; 1];
k = 0.005; % lift coefficient
b = 0.0001; % drag coefficient
l = 0.5; % Length of arm



% Initial state [x, y, z, u, v, w, phi, theta, psi, p, q, r]
initial_state = [0; 0; 10; 0; 0; 0; 0; 0; 0; 0; 0; 0];

% Simulation time
tspan = linspace(0,10, 100);

% Solve ODEs using ode45
[t, states] = ode45(@(t, state) quadcopter_dynamics(t, state, mass, gravity, inertia, omega, k, b, l), tspan, initial_state);

% Extracting states 
x = states(:, 1);
y = states(:, 2);
z = states(:, 3);


figure;
plot3(x, y, z);
title('Trajectory');
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');
grid on;




