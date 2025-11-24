# Fitness Evaluation

## Overview

Fitness evaluation compares SLAM trajectories against ground truth from Isaac Sim.

## Metrics

### Localization Error

The primary metric for attack effectiveness is the localization error between the SLAM-estimated trajectory and the ground truth trajectory.

### Perturbation Magnitude

The imperceptibility objective is measured using the L2 norm of the perturbations applied to the point cloud.

## Evaluation Pipeline

1. Apply perturbations to point cloud
2. Run MOLA SLAM on perturbed data
3. Extract estimated trajectory
4. Compare with ground truth trajectory
5. Compute fitness metrics

## Implementation Details

Coming soon - detailed implementation of the fitness evaluation system.
