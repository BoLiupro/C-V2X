<div align="center">

# C-V2X Positioning Simulation
### Synchronous positioning under weak-GNSS scenarios

![MATLAB](https://img.shields.io/badge/MATLAB-Simulation-orange)
![Topic](https://img.shields.io/badge/C--V2X-Positioning-blue)

</div>

## Overview

This repository contains a MATLAB-based simulation project for studying **C-V2X-assisted positioning under weak GNSS conditions**. The project explores how roadside infrastructure and complementary sensing can improve positioning robustness when satellite signals are degraded.

## Main Components

- `SPP.m` — single-point positioning baseline.
- `CV2X_RTK.m` — C-V2X / RTK-oriented positioning simulation.
- `CV2X_Kalman.m` — Kalman-filter-based state estimation and fusion.
- `Kalman.m` — Kalman filtering utilities.
- `IMU.m` — inertial measurement simulation.
- `RSU.m` — roadside-unit related modeling.
- `Laser_DEM.m` — auxiliary ranging / environmental modeling.
- `NoiseSet.m` and `Gaussian.m` — noise generation and simulation settings.
- `primaryBSSelection.m` — primary base-station selection logic.
- `RepeatExp.m` — repeated experiment evaluation.

## Research Goal

The project studies positioning performance in scenarios where GNSS measurements are weak or unreliable. The simulation combines multiple information sources and compares alternative positioning/fusion strategies to understand the contribution of C-V2X infrastructure and filtering methods.

## Requirements

- MATLAB

No additional package manager is required for the core `.m` scripts. Depending on your MATLAB version and local setup, some toolbox functions may need to be replaced or adapted.

## Usage

Start from the main experiment scripts such as:

```matlab
CV2X_RTK
CV2X_Kalman
RepeatExp
```

Before running experiments, check the parameter definitions and simulation settings in the auxiliary scripts.

## Notes

This repository is an earlier research/project implementation kept for reproducibility and portfolio purposes. The code reflects the experimental setup used at the time and may require minor path or parameter adjustments on a new machine.

## Author

**Bo Liu**  
Hunan University  
Contact: `liubo317@hnu.edu.cn`
