# API Reference

## Overview

This document provides an API reference for the main modules in the MOLA Adversarial NSGA-II project.

## Modules

### Perturbation Generator

::: src.perturbations.perturbation_generator.PerturbationGenerator
    options:
      show_source: false
      heading_level: 4

**Location:** `src/perturbations/perturbation_generator.py`

The `PerturbationGenerator` class implements state-of-the-art adversarial perturbation techniques for LiDAR point clouds.

#### Key Methods

| Method | Description |
|--------|-------------|
| `get_genome_size()` | Returns genome size (17 parameters) |
| `encode_perturbation(genome)` | Converts genome to perturbation parameters |
| `apply_perturbation(cloud, params, seed)` | Applies perturbation to point cloud |
| `compute_chamfer_distance(cloud1, cloud2)` | Computes Chamfer distance between clouds |
| `compute_curvature(points)` | Computes local curvature for targeting |
| `detect_edges_and_corners(points)` | Detects edge/corner points (SLACK-inspired) |
| `reset_temporal_state()` | Resets temporal drift state |

#### Genome Structure (17 parameters)

| Index | Parameter | Range | Description |
|-------|-----------|-------|-------------|
| 0-2 | Noise direction | [-1, 1] | Directional bias for noise |
| 3 | Noise intensity | [0, 1] | Scaled by `noise_std` |
| 4 | Curvature strength | [0, 1] | High-curvature targeting |
| 5 | Dropout rate | [0, 1] | Scaled by `max_dropout_rate` |
| 6 | Ghost ratio | [0, 1] | Scaled by `max_ghost_points_ratio` |
| 7-9 | Cluster direction | [-1, 1] | Direction for cluster perturbation |
| 10 | Cluster strength | [0, 1] | Cluster perturbation intensity |
| 11 | Spatial correlation | [0, 1] | Correlation of nearby perturbations |
| 12 | Geometric distortion | [0, 1] | ICP attack strength |
| 13 | Edge attack | [0, 1] | SLACK-inspired edge targeting |
| 14 | Temporal drift | [0, 1] | Accumulating drift strength |
| 15 | Scanline perturbation | [0, 1] | ASP-inspired attack |
| 16 | Strategic ghost | [0, 1] | Feature-based ghost placement |

---

### Metrics

**Location:** `src/evaluation/metrics.py`

Functions for computing fitness metrics.

#### Functions

| Function | Description |
|----------|-------------|
| `compute_localization_error(gt, est, method)` | Compute ATE/RPE between trajectories |
| `compute_imperceptibility(orig, pert, method)` | Compute perturbation magnitude |
| `compute_multi_objective_fitness(...)` | Combined fitness for NSGA-II |
| `normalize_fitness(values, ref_point)` | Normalize fitness values |

#### ATE Computation

The `_compute_ate` function implements standard Absolute Trajectory Error:

1. **Umeyama alignment**: Rigid alignment (R + t, no scale) of estimated to ground truth
2. **RMSE**: Root mean squared error of per-pose distances

```python
from src.evaluation.metrics import compute_localization_error

ate = compute_localization_error(
    ground_truth_trajectory,  # (N, 3) array
    estimated_trajectory,     # (M, 3) array
    method="ate"              # "ate", "rpe", or "final"
)
```

---

### Data Loaders

**Location:** `src/utils/data_loaders.py`

Functions for loading point clouds and trajectories.

#### Functions

| Function | Description |
|----------|-------------|
| `load_point_clouds_from_npy(path)` | Load point cloud sequence |
| `load_timestamps_from_npy(path)` | Load frame timestamps |
| `load_trajectory_from_tum(path, ...)` | Load trajectory (TUM or NPY format) |

---

### NSGA-III Optimizer

**Location:** `src/optimization/run_nsga2.py`

Main optimization script using pymoo's NSGA-III algorithm.

#### Key Classes

**MOLAEvaluator**: ROS2 node that evaluates genomes by running MOLA SLAM.

```python
evaluator = MOLAEvaluator(
    perturbation_generator=generator,
    ground_truth_trajectory=gt_traj,
    point_cloud_sequence=clouds,
    timestamps=timestamps,
    tf_sequence=tf_data,
    tf_static=tf_static_data,
    mola_binary_path="/opt/ros/jazzy/bin/mola-cli",
    mola_config_path="path/to/config.yaml",
)

# Evaluate a genome
neg_ate, chamfer = evaluator.evaluate(genome)
```

#### Command Line Arguments

```bash
python src/optimization/run_nsga2.py \
    --gt-traj maps/ground_truth_interpolated.npy \
    --frames data/frame_sequence.npy \
    --pop-size 10 \
    --n-gen 20 \
    --max-point-shift 0.05 \
    --noise-std 0.015 \
    --max-dropout 0.15 \
    --output src/results/optimized_genome.npy
```

---

### Plotting

**Location:** `src/plots/plot_nsga2_results.py`

Visualization of optimization results.

```bash
python src/plots/plot_nsga2_results.py src/results/optimized_genome6
```

Generates:
- Pareto front plot (ATE vs Chamfer distance)
- Best solution analysis
- Parameter correlation analysis

---

## Usage Examples

### Basic Perturbation

```python
import numpy as np
from src.perturbations.perturbation_generator import PerturbationGenerator

# Create generator
gen = PerturbationGenerator(
    max_point_shift=0.05,  # 5cm
    noise_std=0.02,        # 2cm
    max_dropout_rate=0.15  # 15%
)

# Generate random genome
genome = gen.random_genome()

# Apply perturbation
params = gen.encode_perturbation(genome)
perturbed = gen.apply_perturbation(point_cloud, params, seed=42)

# Measure imperceptibility
chamfer = gen.compute_chamfer_distance(point_cloud, perturbed)
```

### Running Optimization

```python
from pymoo.algorithms.moo.nsga3 import NSGA3
from pymoo.util.ref_dirs import get_reference_directions

# Setup NSGA-III
ref_dirs = get_reference_directions("das-dennis", 2, n_partitions=12)
algorithm = NSGA3(
    pop_size=len(ref_dirs),
    ref_dirs=ref_dirs,
)

# Run optimization
result = minimize(problem, algorithm, ('n_gen', 20), seed=42)

# Get Pareto front
pareto_front = result.F  # Fitness values
pareto_set = result.X    # Genomes
```

---

## References

- **NSGA-III**: Deb & Jain (2014) - Reference-point based NSGA
- **SLACK**: arXiv 2024 - Attacking LiDAR-based SLAM
- **ICP Attack**: arXiv 2403.05666 - ICP adversarial perturbations
- **ASP**: IEEE 2024 - Attribution-based Scanline Perturbation
- **FLAT**: ECCV 2024 - Flux-Aware Imperceptible Adversarial Attacks
