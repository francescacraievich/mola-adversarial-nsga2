# Multi-Objective Adversarial Perturbations for SLAM Systems

Welcome to the documentation for the MOLA Adversarial NSGA-II project.

## Overview

This project applies **NSGA-II** (Non-dominated Sorting Genetic Algorithm II) to evolve adversarial perturbations on LiDAR point clouds that can compromise SLAM (Simultaneous Localization and Mapping) systems.

## Objectives

The algorithm optimizes two competing objectives:

1. **Attack Effectiveness** - Maximize localization error in the SLAM system
2. **Imperceptibility** - Minimize the magnitude of perturbations to make them harder to detect

This creates a Pareto front of optimal trade-offs between these two objectives.

## Key Features

- Multi-objective evolutionary optimization using NSGA-II
- Integration with MOLA SLAM in Isaac Sim
- Automated fitness evaluation using ground truth comparison
- Baseline comparisons (random perturbations, grid search)
- Comprehensive analysis and visualization tools

## Project Context

This project is part of a practical application of multi-objective evolutionary algorithms. The setup includes:

- MOLA SLAM operational in Isaac Sim
- Recorded point clouds from a rover simulation
- Automatic fitness evaluation by comparing SLAM trajectories with simulator ground truth

## Quick Links

- [Installation Guide](getting-started/installation.md)
- [Quick Start](getting-started/quickstart.md)
- [NSGA-II Algorithm Details](user-guide/nsga2.md)
- [API Reference](api/overview.md)

## Repository

View the source code on [GitHub](https://github.com/francescacraievich/mola-adversial-nsga2).
