# Multi-Objective Adversarial Perturbations for SLAM Systems using NSGA-II

Evolutionary multi-objective optimization of adversarial perturbations on LiDAR point clouds to evaluate the robustness of SLAM systems.

## Overview

This project uses **NSGA-II** (Non-dominated Sorting Genetic Algorithm II) to evolve adversarial perturbations on LiDAR point clouds that compromise SLAM systems. The algorithm optimizes a trade-off between:

- **Attack Effectiveness**: Maximize localization error in the SLAM system
- **Imperceptibility**: Minimize the magnitude of perturbations

## Setup

The system is integrated with **MOLA SLAM** running in **Isaac Sim** with recorded point clouds from a rover, enabling automatic fitness evaluation by comparing SLAM trajectories against ground truth from the simulator.

## Features

- Multi-objective optimization using NSGA-II
- Automated fitness evaluation against ground truth
- Pareto-optimal perturbation generation
- Comparison with baseline approaches (random perturbations, grid search)

## Installation

```bash
pip install -r requirements.txt
```

## Usage

Coming soon...

## Project Structure

```
mola-adversial-nsga2/
├── src/              # Source code
├── mola/             # MOLA SLAM integration
├── docs/             # Documentation
└── tests/            # Test suite
```

## Deliverables

- Python implementation (perturbations + NSGA-II + evaluation)
- Comparison with baseline approaches
- Documentation and results analysis

## License

MIT

## Author

Francesca Craievich
