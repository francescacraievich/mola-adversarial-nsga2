# NSGA-II Algorithm

## Overview

NSGA-II (Non-dominated Sorting Genetic Algorithm II) is a multi-objective evolutionary algorithm that maintains a diverse set of solutions along the Pareto front.

## Key Concepts

### Pareto Dominance

A solution **dominates** another if it is better in at least one objective and not worse in any other objective.

### Non-dominated Sorting

Solutions are ranked into fronts based on dominance relationships:

- Front 1: Non-dominated solutions (best)
- Front 2: Solutions dominated only by Front 1
- And so on...

### Crowding Distance

Within each front, solutions are assigned a crowding distance to maintain diversity. Solutions with larger crowding distances (more isolated) are preferred to maintain spread along the Pareto front.

## Algorithm Steps

1. **Initialize** population randomly
2. **Evaluate** fitness for all individuals
3. **Non-dominated sorting** to assign ranks
4. **Calculate crowding distance** within each front
5. **Selection** using tournament selection based on rank and crowding distance
6. **Crossover** and **mutation** to create offspring
7. **Combine** parent and offspring populations
8. **Select** best individuals for next generation
9. Repeat steps 2-8 until termination

## Application to Adversarial Perturbations

In this project:

- **Individual**: A set of perturbations to apply to point cloud points
- **Objective 1**: Maximize SLAM localization error
- **Objective 2**: Minimize perturbation magnitude (L2 norm)

The result is a set of Pareto-optimal perturbations offering different trade-offs between attack effectiveness and imperceptibility.
