# NSGA-III Algorithm

## Overview

This project uses **NSGA-III** (Non-dominated Sorting Genetic Algorithm III), an advanced multi-objective evolutionary algorithm designed for better handling of many-objective optimization problems. NSGA-III improves upon NSGA-II by using reference-point based selection instead of crowding distance, providing better diversity maintenance along the Pareto front.

In this project, we optimize two competing objectives:
1. **Maximize SLAM localization error** (attack effectiveness)
2. **Minimize perturbation magnitude** (imperceptibility)

Unlike single-objective optimization that finds one best solution, NSGA-III finds a set of Pareto-optimal solutions that represent different trade-offs between the objectives.

## Why Multi-Objective Optimization?

In adversarial attacks on SLAM systems, we face two competing objectives:

1. **Maximize ATE (Absolute Trajectory Error)** - We want to degrade MOLA's localization accuracy as much as possible
2. **Minimize perturbation magnitude** - We want perturbations to be small and imperceptible

You cannot optimize both simultaneously because:
- Larger perturbations cause more damage but are easier to detect
- Smaller perturbations are stealthier but cause less damage

NSGA-II finds the Pareto front: the set of solutions where improving one objective requires worsening the other.

## Key Concepts

### Pareto Dominance

Solution A **dominates** solution B if:
- A is better than B in at least one objective
- A is not worse than B in any objective

Example:
- Solution A: ATE = 1.5m, perturbation = 2cm
- Solution B: ATE = 1.2m, perturbation = 3cm
- A dominates B (higher ATE and lower perturbation)

### Pareto Front

The Pareto front is the set of non-dominated solutions. These are the "best" solutions where you cannot improve one objective without worsening another.

In our case, the Pareto front shows:
- Minimum perturbation needed to achieve a given ATE
- Maximum ATE achievable with a given perturbation budget

### Non-dominated Sorting

NSGA-II ranks solutions into fronts:
- **Front 1**: Non-dominated solutions (the Pareto front)
- **Front 2**: Solutions dominated only by Front 1
- **Front 3**: Solutions dominated only by Fronts 1 and 2
- And so on...

During selection, Front 1 solutions are preferred, then Front 2, etc.

### Reference Points (NSGA-III)

NSGA-III uses **reference points** instead of crowding distance for diversity maintenance. Reference points are uniformly distributed on a normalized hyperplane and guide the search toward well-spread solutions.

**Das-Dennis method**: We use Das-Dennis reference directions to generate evenly-spaced reference points:
- For 2 objectives with 12 partitions: generates 13 reference points
- Each solution is associated with its nearest reference point
- Selection favors solutions in underrepresented regions

This provides better diversity than crowding distance, especially for problems with 3+ objectives.

### Crowding Distance (Legacy - NSGA-II)

NSGA-II used crowding distance to maintain diversity:

- **Large crowding distance**: Solution is in a sparse region (preferred)
- **Small crowding distance**: Solution has many neighbors (less preferred)

NSGA-III's reference-point approach is generally more effective for maintaining spread along the Pareto front.

## Algorithm Steps

### 1. Initialization
Generate an initial population of random solutions. Each solution is a "genome" encoding perturbations for the point cloud.

In our implementation:
- Population size: 10-13 individuals (based on reference directions)
- Reference partitions: 12 (Das-Dennis)
- Each genome has **17 parameters** encoding multiple attack strategies

### 2. Fitness Evaluation
Evaluate each individual by:
1. Apply perturbations to point clouds
2. Run MOLA SLAM on perturbed data
3. Compare estimated trajectory with ground truth
4. Calculate ATE (objective 1) and perturbation magnitude (objective 2)

This is the most computationally expensive step, taking several minutes per evaluation.

### 3. Non-dominated Sorting
Rank all solutions into fronts based on dominance relationships. Solutions in Front 1 are the current best approximation of the Pareto front.

### 4. Crowding Distance Calculation
For each front, calculate crowding distance to identify which solutions maintain diversity along the front.

### 5. Tournament Selection
Select parents for reproduction using tournament selection:
1. Randomly pick two individuals
2. Select the one with better rank (lower front number)
3. If tied, select the one with larger crowding distance

This favors both quality (low rank) and diversity (high crowding distance).

### 6. Crossover and Mutation
Create offspring through genetic operators:

**Crossover**: Combine two parent genomes
- Example: Take attack strategy from parent A, parameters from parent B
- Crossover probability: 90%

**Mutation**: Randomly modify genome components
- Example: Change dropout rate from 5% to 7%
- Mutation probability: 10%

### 7. Combine Populations
Merge parent population (size N) with offspring population (size N) to create a combined pool of size 2N.

### 8. Environmental Selection
Select the best N individuals for the next generation:
1. Sort combined population into fronts
2. Add entire fronts to next generation until full
3. If the last front doesn't fit completely, use crowding distance to select the most diverse individuals

### 9. Termination
Repeat steps 2-8 for a fixed number of generations (typically 20 generations).

Final output is Front 1 from the last generation, which approximates the true Pareto front.

## Parameters

Key parameters in our NSGA-III implementation:

- **Population size**: 10-13 individuals (determined by reference directions)
- **Reference partitions**: 12 (Das-Dennis method)
- **Number of generations**: 20 (for ~200 total evaluations)
- **Crossover probability**: 0.9 (SBX crossover, eta=15)
- **Mutation probability**: Polynomial mutation (eta=20)

For quick testing, use smaller values:
- Population size: 4
- Generations: 5
- Reference partitions: 3

## Genome Encoding (17 Parameters)

Each individual is encoded as a **17-parameter genome** that combines multiple attack strategies inspired by recent research (SLACK, ICP Attack, ASP, FLAT):

| Index | Parameter | Range | Description |
|-------|-----------|-------|-------------|
| 0-2 | Noise direction | [-1, 1] | Directional bias for noise (x, y, z) |
| 3 | Noise intensity | [0, 1] | Scaled by `noise_std` config |
| 4 | Curvature strength | [0, 1] | High-curvature point targeting |
| 5 | Dropout rate | [0, 1] | Scaled by `max_dropout_rate` |
| 6 | Ghost ratio | [0, 1] | Scaled by `max_ghost_points_ratio` |
| 7-9 | Cluster direction | [-1, 1] | Direction for cluster perturbation |
| 10 | Cluster strength | [0, 1] | Cluster perturbation intensity |
| 11 | Spatial correlation | [0, 1] | Correlation of nearby perturbations |
| 12 | **Geometric distortion** | [0, 1] | Range-dependent scaling (KEY for ICP) |
| 13 | **Edge attack** | [0, 1] | SLACK-inspired edge/corner targeting |
| 14 | **Temporal drift** | [0, 1] | Accumulating drift (breaks loop closure) |
| 15 | **Scanline perturbation** | [0, 1] | ASP-inspired along-beam perturbation |
| 16 | **Strategic ghost** | [0, 1] | Feature-based ghost point placement |

### Attack Combinations

The genome allows combining multiple strategies simultaneously:

- **High ATE, High Detectability**: Maximize all parameters
- **Balanced Attack**: Moderate temporal drift + edge attack
- **Stealthy Attack**: Low intensity across all parameters

The genome is represented as a numpy array in range [-1, 1] that pymoo manipulates with genetic operators.

## Constraint Handling

Physical constraints are enforced:
- **Maximum perturbation per point**: 50cm (points moved beyond this are unrealistic)
- **Dropout rate**: 0-100% (cannot remove more than all points)
- **Noise standard deviation**: 0-10cm (larger values create obvious artifacts)

Constraint violations are handled by:
1. Returning infinite fitness (solution is automatically excluded from Pareto front)
2. Repairing infeasible solutions to nearest feasible point

## Convergence

NSGA-II typically converges to a good approximation of the Pareto front within 20-50 generations. You can monitor convergence by tracking:

1. **Hypervolume**: Volume of objective space dominated by Pareto front (should increase)
2. **Best ATE**: Maximum ATE achieved (should increase)
3. **Best efficiency**: Highest ATE per cm of perturbation (should increase)

In practice, we observe:
- Baseline ATE: 68cm (unperturbed)
- After 20 generations: 120-150cm ATE with 5-10cm perturbations
- Convergence plateaus around generation 15-20

## Comparison with Single-Objective Optimization

Why not use single-objective optimization (e.g., maximize ATE only)?

**Single-objective approach:**
- Would find solutions with very high ATE but also very large perturbations
- No control over perturbation budget
- Misses efficient solutions with good ATE/perturbation trade-offs

**NSGA-II multi-objective approach:**
- Finds entire Pareto front with diverse trade-offs
- Allows choosing solution based on perturbation budget constraints
- Identifies most efficient attack strategies (highest ATE per cm)
- Discovers that dropout is more efficient than noise or ghost points

## Interpreting Results

After optimization, the Pareto front shows:

**Example solutions:**
1. Solution A: ATE = 0.95m, perturbation = 1cm (conservative attack)
2. Solution B: ATE = 1.30m, perturbation = 5cm (balanced attack)
3. Solution C: ATE = 1.65m, perturbation = 12cm (aggressive attack)

**Key insight:** Dropout at 5% is most efficient
- Achieves 18.9% ATE increase per cm of perturbation
- Outperforms Gaussian noise (15.2%), feature targeting (12.7%), and ghost points (8.3%)
- Removing points breaks feature correspondences and degrades loop closure

## Advantages of NSGA-II

1. **No weight tuning**: Unlike weighted sum approaches, NSGA-II doesn't require manually tuning weights for objectives
2. **Diverse solutions**: Maintains multiple solutions with different trade-offs
3. **Robust**: Works well even when objectives have different scales or units
4. **Parallelizable**: Fitness evaluations are independent and can be parallelized

## Limitations

1. **Computational cost**: Requires hundreds of fitness evaluations, each taking minutes
2. **Stochastic**: Results vary between runs due to randomness
3. **Scaling**: Performance degrades with >3 objectives (not an issue here)
4. **Local optima**: May get stuck in local Pareto fronts, though crossover helps escape

## Implementation Details

Our implementation uses [pymoo](https://pymoo.org/), a Python framework for multi-objective optimization:

```python
from pymoo.algorithms.moo.nsga3 import NSGA3
from pymoo.util.ref_dirs import get_reference_directions
from pymoo.optimize import minimize

# Generate Das-Dennis reference directions
ref_dirs = get_reference_directions("das-dennis", 2, n_partitions=12)

algorithm = NSGA3(
    pop_size=len(ref_dirs),  # Population size from reference directions
    ref_dirs=ref_dirs,
)

result = minimize(
    problem,
    algorithm,
    ('n_gen', 20),
    seed=42,
    verbose=True
)

# Access results
pareto_front = result.F  # Fitness values (N, 2)
pareto_set = result.X    # Genomes (N, 17)
```

The problem definition includes:
- Number of variables: 17 (genome length)
- Variable bounds: [-1, 1] for all parameters
- Number of objectives: 2 (negative ATE, Chamfer distance)
- Evaluation function: applies perturbations and runs MOLA SLAM

## NSGA-III vs NSGA-II

| Feature | NSGA-II | NSGA-III |
|---------|---------|----------|
| Diversity mechanism | Crowding distance | Reference points |
| Scalability | Good for 2-3 objectives | Better for many objectives |
| Parameter tuning | Minimal | Reference partitions |
| Pareto front coverage | Can have gaps | More uniform |

We use NSGA-III because:
1. Better diversity maintenance along the Pareto front
2. More consistent results across runs
3. Reference-point approach is more principled

## Further Reading

- Original NSGA-III paper: Deb & Jain (2014) - "An Evolutionary Many-Objective Optimization Algorithm Using Reference-Point-Based Nondominated Sorting Approach"
- Original NSGA-II paper: Deb et al. (2002)
- pymoo documentation: https://pymoo.org/
- Das-Dennis reference directions: Das & Dennis (1998)
