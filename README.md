# 3-DOF Robotic Manipulator Forward & Inverse Kinematics with Trajectory Interpolation

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b%20%7C%20R2024b-0076A8?logo=mathworks&logoColor=white)](https://www.mathworks.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Domain](https://img.shields.io/badge/Domain-Robotics%20&%20Multibody%20Dynamic%20Simulation-lightgrey.svg)](#)

A standalone MATLAB implementation of 3-DOF Robotic Manipulator Forward. Includes the governing dynamics, analytical formulations, and an executable script you can run directly without proprietary third-party dependencies.

## Overview

This repository provides analytical forward and inverse kinematics for a 3-DOF planar robotic arm. It decouples the wrist position and calculates joint angles to track target Cartesian coordinates without numerical singularity issues.

## Governing Equations & Mathematical Formulation

### Forward Kinematics Position Mapping

$$
x_e = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2) + L_3 \cos(\theta_1 + \theta_2 + \theta_3)
$$

$$
y_e = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2) + L_3 \sin(\theta_1 + \theta_2 + \theta_3)
$$

### Differential Kinematics & Jacobian

$$
\dot{X}_e = J(\Theta) \dot{\Theta}
$$

where $L_1, L_2, L_3$ are link lengths and $\theta_1, \theta_2, \theta_3$ are joint angles relative to the reference frame.

## Getting Started

### Prerequisites
- MATLAB (tested on R2022b through R2024b)
- Standard base MATLAB installation (no paid external toolboxes required for this starter script)

### Running the Code
1. Clone the repository:
   ```bash
   git clone https://github.com/matlabsolutions-simulink/robot-manipulator-inverse-kinematics-simscape.git
   cd robot-manipulator-inverse-kinematics-simscape
   ```
2. Open MATLAB, navigate to the cloned folder, and run:
   ```matlab
   run_robot_manipulator_ik
   ```

## Need the Complete Simulink or Simscape Model?

If you are working on a university capstone, thesis, or lab assignment and need the complete `.slx` model with Simscape physical networks, custom parameter lookup tables, or automated test harnesses, our team at [MATLABSolutions.com](https://www.matlabsolutions.com/order-now.php?ref=github_robot_manipulator_inverse_kinematics_simscape) provides custom academic simulation and consulting support.

## Technical Inquiries & Contact
- Website: [matlabsolutions.com](https://www.matlabsolutions.com)
- Custom Consulting Portal: [matlabsolutions.com/order-now.php](https://www.matlabsolutions.com/order-now.php)
- Email: info@matlabsolutions.com

## License
This project is open-source under the [MIT License](LICENSE).
