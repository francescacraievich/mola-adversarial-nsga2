# Multi-Objective Adversarial Perturbations for SLAM Systems

Welcome to the documentation for the MOLA Adversarial NSGA-III project.

## Overview

This project applies **NSGA-III** (Non-dominated Sorting Genetic Algorithm III) to evolve adversarial perturbations on LiDAR point clouds that can compromise SLAM (Simultaneous Localization and Mapping) systems. The project implements state-of-the-art adversarial attack techniques inspired by recent research:

- **FLAT**: Flux-Aware Imperceptible Adversarial Attacks (ECCV 2024)
- **SLACK**: Attacking LiDAR-based SLAM (arXiv 2024)
- **ICP Attack**: Adversarial attacks on ICP registration (arXiv 2403.05666)
- **ASP**: Attribution-based Scanline Perturbation (IEEE 2024)

## Objectives

The algorithm optimizes two competing objectives:

1. **Attack Effectiveness** - Maximize localization error (ATE) in MOLA SLAM
2. **Imperceptibility** - Minimize perturbation magnitude (Chamfer distance)

This creates a Pareto front of optimal trade-offs between attack effectiveness and detectability.

## Key Features

- **NSGA-III optimization** with Das-Dennis reference directions
- **17-parameter genome** combining multiple attack strategies
- **Advanced attacks**: Edge targeting, temporal drift, scanline perturbation, geometric distortion
- Integration with **MOLA SLAM** in Isaac Sim
- Automated fitness evaluation using ground truth comparison
- Comprehensive analysis and visualization tools

## Attack Strategies

The 17-parameter genome encodes:

| Attack Type | Description | Effectiveness |
|-------------|-------------|---------------|
| Dropout | Remove random points | High (18.9% ATE/cm) |
| Gaussian noise | Add random displacement | Medium (15.2% ATE/cm) |
| Edge attack | Target ICP-critical features | High |
| Temporal drift | Accumulating bias (breaks loop closure) | Very High |
| Geometric distortion | Range-dependent scaling | High |
| Scanline perturbation | Along-beam perturbation | Medium |
| Strategic ghost | Feature-based ghost points | Medium |

## Project Context

This project is part of a practical application of multi-objective evolutionary algorithms. The setup includes:

- MOLA SLAM operational in Isaac Sim
- Recorded point clouds from a simulated rover (~11.3s, 113 frames at 10Hz)
- Automatic fitness evaluation using Umeyama-aligned ATE
- Baseline ATE: ~6.8cm (unperturbed)

## Quick Links

- [Installation Guide](getting-started/installation.md)
- [Quick Start](getting-started/quickstart.md)
- [NSGA-III Algorithm Details](user-guide/nsga2.md)
- [Perturbation Strategies](user-guide/perturbations.md)
- [API Reference](api/overview.md)
- [Experimental Results](results/experiments.md)

## Repository

View the source code on [GitHub](https://github.com/francescacraievich/mola-adversarial-nsga2).
