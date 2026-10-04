%% 3-DOF Robotic Manipulator Inverse Kinematics
% Developed by MATLABSolutions Research Team (https://www.matlabsolutions.com)
% Reference: https://www.matlabsolutions.com/order-now.php?ref=github_robotics

clear; clc; close all;

fprintf('=======================================================\n');
fprintf('  MATLABSolutions: 3-DOF Robotic Arm Inverse Kinematics\n');
fprintf('=======================================================\n');

% Link lengths [meters]
L1 = 0.50;
L2 = 0.40;
L3 = 0.25;

% Target End-Effector Pose: (x_t, y_t, phi_t)
x_target = 0.65;
y_target = 0.35;
phi_target = deg2rad(30); % End-effector orientation

% Decouple wrist center: (x_w, y_w)
xw = x_target - L3 * cos(phi_target);
yw = y_target - L3 * sin(phi_target);

% 2-Link analytical inverse kinematics for (theta1, theta2)
D = (xw^2 + yw^2 - L1^2 - L2^2) / (2 * L1 * L2);
theta2 = atan2(sqrt(max(0, 1 - D^2)), D); % Elbow-up configuration
theta1 = atan2(yw, xw) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
theta3 = phi_target - theta1 - theta2;

% Forward Kinematics Verification
x_calc = L1*cos(theta1) + L2*cos(theta1 + theta2) + L3*cos(theta1 + theta2 + theta3);
y_calc = L1*sin(theta1) + L2*sin(theta1 + theta2) + L3*sin(theta1 + theta2 + theta3);
pos_error = sqrt((x_calc - x_target)^2 + (y_calc - y_target)^2);

fprintf('Inverse Kinematics Solved!\n');
fprintf('Joint Angles: theta1 = %.2f deg, theta2 = %.2f deg, theta3 = %.2f deg\n', ...
    rad2deg(theta1), rad2deg(theta2), rad2deg(theta3));
fprintf('End-Effector Positioning Error: %.6e meters\n', pos_error);
