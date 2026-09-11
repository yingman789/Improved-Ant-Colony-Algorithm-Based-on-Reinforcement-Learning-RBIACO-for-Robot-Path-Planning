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
This will generate a random 20×20 map, run the algorithm, and display the optimal path and convergence curve.

Using Your Own Map
To apply RBIACO to your own grid map:

Prepare a binary matrix grid_1 where 0 = free cell, 1 = obstacle.

Define startPos and goalPos as [row, column] indices (1‑based).

Call the function:

matlab
[bestPath, bestLen, convIter, runTime, bestHistory] = RBIACO_RealTime(grid_1, startPos, goalPos);
Output Interpretation
bestPath : Matrix of waypoints (row, col) for the best path found.

bestLen : Total Euclidean length of the best path.

convIter: The iteration index at which the final best solution was first discovered.

runTime : Total execution time (seconds).

bestHistory: A vector of the best path length recorded at each iteration (useful for convergence plots).

Example
matlab
% Load or create your map (here we use a random 30×30 map)
grid_1 = double(rand(30,30) > 0.7);
grid_1(1,1) = 0; grid_1(30,30) = 0;
[startPos, goalPos] = deal([1,1], [30,30]);

% Run algorithm
[path, len, iter, time, hist] = RBIACO_RealTime(grid_1, startPos, goalPos);

% Display result
fprintf('Path length: %.2f, found at iteration %d, time: %.2f s\n', len, iter, time);
Code Structure
RBIACO_RealTime.m – The main algorithm function with extensive comments.

run_demo.m – A self-contained demo script that generates a random map, runs the algorithm, and plots results.

README.md – This file.

Performance Notes
The algorithm is optimized for maps up to about 100×100. For larger maps, consider increasing maxIter or adjusting parameters.

The Q‑value update has been implemented using a suffix‑distance technique to reduce complexity from O(L²) to O(L) per path.

Contact
For questions or bug reports, please open an issue on GitHub or contact the corresponding author (see paper).

### Summary of Changes for GitHub Release

| Aspect | Implementation |
|--------|----------------|
| **Parameter transparency** | All parameters are explicitly defined and commented in the code; a dedicated table is included in the README. |
| **Code documentation** | Every major section is explained with detailed English comments, including references to equations in the paper. |
| **Reproducibility** | The demo script uses a fixed random seed for reproducible results. |
| **Ease of use** | A single function call with clear inputs/outputs; the demo shows how to integrate it. |
| **Guidance** | Step‑by‑step instructions in the README cover installation, parameter tuning, and result interpretation. |

You can now upload these three files (`RBIACO_RealTime.m`, `run_demo.m`, `README.md`) to your GitHub repository. The reviewer will find all required information to replicate your experiments.
