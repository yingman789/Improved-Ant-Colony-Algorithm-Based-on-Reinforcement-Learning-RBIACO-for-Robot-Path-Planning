%% run_demo.m - Demonstration of RBIACO_RealTime on a random map
% This script generates a 20x20 grid map with obstacles, runs the RBIACO
% algorithm, and displays the resulting path and convergence curve.

clear; clc; close all;
rng(42);  % Fix random seed for reproducibility (optional)

% -------- Map configuration --------
gridSize = [20, 20];
startPos = [1, 1];
goalPos = [20, 20];
obstacleProb = 0.25;   % Probability of a cell being obstacle

% Generate random binary map
grid_1 = zeros(gridSize);
grid_1(rand(gridSize) < obstacleProb) = 1;
grid_1(startPos(1), startPos(2)) = 0;
grid_1(goalPos(1), goalPos(2)) = 0;

% -------- Run the RBIACO algorithm --------
fprintf('Running RBIACO on a %dx%d map...\n', gridSize(1), gridSize(2));
[bestPath, bestLen, convIter, runTime, bestHistory] = ...
    RBIACO_RealTime(grid_1, startPos, goalPos);

% -------- Display results --------
fprintf('Optimal path length: %.4f\n', bestLen);
fprintf('Convergence iteration: %d\n', convIter);
fprintf('Runtime: %.4f seconds\n', runTime);

% -------- Plot path --------
figure('Name', 'RBIACO Path Planning', 'NumberTitle', 'off');
imagesc(grid_1);
colormap([1 1 1; 0.5 0.5 0.5]);
hold on;
plot(startPos(2), startPos(1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(goalPos(2), goalPos(1), 'ro', 'MarkerSize', 12, 'LineWidth', 2);
if ~isempty(bestPath)
    plot(bestPath(:,2), bestPath(:,1), 'b-', 'LineWidth', 2.5);
    plot(bestPath(:,2), bestPath(:,1), 'b.', 'MarkerSize', 8);
end
title(sprintf('RBIACO Optimal Path (length = %.2f)', bestLen));
xlabel('Column'); ylabel('Row');
axis equal tight;
grid on;
set(gca, 'YDir', 'normal');

% -------- Plot convergence curve --------
figure('Name', 'Convergence Curve', 'NumberTitle', 'off');
plot(1:length(bestHistory), bestHistory, 'b-', 'LineWidth', 2);
xlabel('Iteration'); ylabel('Optimal path length');
title('RBIACO Convergence Curve');
grid on;

disp('All figures displayed.');