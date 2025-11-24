"""
NSGA-II (Non-dominated Sorting Genetic Algorithm II) implementation.

Multi-objective evolutionary algorithm for generating Pareto-optimal solutions.
"""

from .nsga2 import NSGA2
from .operators import tournament_selection, simulated_binary_crossover, polynomial_mutation
from .utils import fast_non_dominated_sort, crowding_distance

__all__ = [
    "NSGA2",
    "tournament_selection",
    "simulated_binary_crossover",
    "polynomial_mutation",
    "fast_non_dominated_sort",
    "crowding_distance"
]
