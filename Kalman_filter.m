%% MATLABSolutions: Kalman Filter 1D Vehicle Tracker
% Demonstrates how visual Gaussian multiplication maps to real code
clear; clc; close all;

% Simulation Parameters
dt = 0.1;           % Time step (seconds)
t = 0:dt:10;        % Total duration
N = length(t);

% True State: Position and Velocity (Constant velocity motion)
true_velocity = 2.0;               % 2 m/s
true_pos = true_velocity * t;

% Process and Measurement Noise
sigma_process = 0.2;               % Model uncertainty (wheel slip)
sigma_meas = 1.5;                  % GPS measurement noise

noisy_meas = true_pos + sigma_meas * randn(1, N);

% Kalman Filter Initialization
x_est = 0;                         % Initial position estimate
P_est = 2.0;                       % Initial uncertainty (variance)

estimates = zeros(1, N);
uncertainties = zeros(1, N);

for k = 1:N
    % 1. PREDICT: Project state and covariance forward
    x_pred = x_est + true_velocity * dt;
    P_pred = P_est + (sigma_process^2);
    
    % 2. UPDATE: Compute Kalman Gain and fuse sensor reading
    K = P_pred / (P_pred + sigma_meas^2);          % Kalman Gain
    x_est = x_pred + K * (noisy_meas(k) - x_pred); % Optimal State Update
    P_est = (1 - K) * P_pred;                      % Variance shrinks!
    
    estimates(k) = x_est;
    uncertainties(k) = P_est;
end

% Plotting results
figure('Color', 'w');
plot(t, true_pos, 'k--', 'LineWidth', 2, 'DisplayName', 'True Trajectory'); hold on;
plot(t, noisy_meas, 'ro', 'MarkerSize', 4, 'DisplayName', 'Noisy Sensor (GPS)');
plot(t, estimates, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Kalman Filter Estimate');
xlabel('Time (s)'); ylabel('Position (m)');
title('1D Tracking: The Kalman Filter in Action');
legend('Location', 'NorthWest'); grid on;