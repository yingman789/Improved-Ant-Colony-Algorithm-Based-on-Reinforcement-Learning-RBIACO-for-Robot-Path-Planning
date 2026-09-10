# Improved-Ant-Colony-Algorithm-Based-on-Reinforcement-Learning-RBIACO-for-Robot-Path-Planning
This algorithm is entirely written in MATLAB code, further addressing the shortcomings of ant colony algorithm, including slow convergence speed, susceptibility to premature local optima and deadlocks, and poor robustness.

This repository contains the MATLAB implementation of **RBIACO** (Reinforcement Learning-Based Improved Ant Colony Optimization), a novel path planning algorithm for mobile robots in grid environments. The algorithm integrates Q-learning with ant colony optimization to achieve faster convergence, better solution quality, and robust deadlock avoidance.

## Features
- **Exploration auxiliary factor** to prevent deadlocks and encourage exploration of unvisited areas.
- **Differentiated pheromone increment** that assigns higher reinforcement to nodes on the optimal path.
- **Node value estimation** derived from the Bellman equation to refine pheromone updates.
- **Random perturbation** to escape local optima.
- Optimized for real-time performance via precomputed neighbor tables and sparse V updates.
- Self-contained MATLAB implementation with detailed comments.

## Requirements
- MATLAB R2020b or later (tested on R2023b)
- No additional toolboxes required.

## Parameter Settings
All algorithm parameters are defined at the top of the `RBIACO_RealTime` function. You can adjust them as needed:

| Parameter | Symbol | Default Value | Description |
|-----------|--------|---------------|-------------|
| Number of ants | `numAnts` | 30 | Population size per iteration |
| Max iterations | `maxIter` | 100 | Termination criterion |
| Pheromone weight | `alpha` | 1 | Influence of pheromone on selection (Eq.7) |
| Heuristic weight | `beta`  | 2 | Influence of heuristic (inverse distance) (Eq.7) |
| Exploration weight | `gamma` | 2 | Influence of exploration factor (Eq.7) |
| Evaporation rate | `rho`   | 0.05 | Pheromone decay rate (Eq.18) |
| Greedy threshold | `p0`    | 0.2 | Probability of greedy selection (Eq.8) |
| Discount factor | `b_discount` | 0.6 | Q-learning discount factor (Eq.14) |
| Learning weight | `lambda` | 0.6 | Weight for Q-value update (Eq.13) |

These values have been experimentally tuned for 20×20 to 40×40 maps. You may need to adjust them for your specific environment.

## Usage

### Quick Start
1. Clone or download this repository.
2. Open MATLAB and navigate to the folder.
3. Run the demo script:
   ```matlab
   run_demo
