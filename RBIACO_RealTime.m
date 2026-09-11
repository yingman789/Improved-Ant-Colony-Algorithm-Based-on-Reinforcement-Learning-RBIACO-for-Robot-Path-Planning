function [bestPath, bestLen, convIter, runTime, bestHistory] = RBIACO_RealTime(grid_1, startPos, goalPos)
% ========================================================================
% RBIACO_RealTime - Reinforcement Learning Enhanced Ant Colony Optimization
%                   for Mobile Robot Path Planning (Real-Time Version)
%
% Description:
%   This function implements the proposed RBIACO algorithm, which integrates
%   reinforcement learning (Q-learning) with ant colony optimization to solve
%   path planning problems in grid maps. It incorporates:
%     - Exploration auxiliary factor (w) to avoid deadlocks
%     - Differentiated pheromone increment (Delta) based on node importance
%     - Node value estimation (V) derived from Bellman equation
%     - Random perturbation to escape local optima
%   The code is optimized for real-time performance using precomputed neighbor
%   tables, sparse V updates, and an efficient Q-value computation.
%
% Inputs:
%   grid_1     - 2D binary matrix (rows x cols), 0 = free, 1 = obstacle
%   startPos   - [row, col] coordinates of the start point (1-indexed)
%   goalPos    - [row, col] coordinates of the goal point
%
% Outputs:
%   bestPath   - [N x 2] matrix of waypoints (row, col) of the best found path
%   bestLen    - scalar, total length of the best path (Euclidean distance)
%   convIter   - iteration index at which the final best solution was first found
%   runTime    - total execution time (seconds)
%   bestHistory - [maxIter x 1] vector of the best path length at each iteration
%
% Parameters (default values, can be modified inside the function):
%   numAnts = 30, maxIter = 100, alpha = 1, beta = 2, gamma = 2,
%   rho = 0.05, p0 = 0.2, b_discount = 0.6, lambda = 0.6
% ========================================================================

    % -------- 1. Map properties and direction vectors --------
    [rows, cols] = size(grid_1);
    dirs = [-1,-1; -1,0; -1,1; 0,-1; 0,1; 1,-1; 1,0; 1,1];  % 8-neighbor moves

    % -------- 2. Precompute neighbor table (for speed) --------
    % For each free cell, store the list of its free neighbors.
    neighborMap = cell(rows, cols);
    for r = 1:rows
        for c = 1:cols
            if grid_1(r,c) == 1, continue; end
            nbrs = [];
            for d = 1:size(dirs,1)
                nr = r + dirs(d,1); nc = c + dirs(d,2);
                if nr>=1 && nr<=rows && nc>=1 && nc<=cols && grid_1(nr,nc)==0
                    nbrs = [nbrs; nr, nc];
                end
            end
            neighborMap{r,c} = nbrs;
        end
    end

    % -------- 3. Algorithm parameters (USER-CONFIGURABLE) --------
    numAnts = 30;          % Number of ants per iteration
    maxIter = 100;         % Maximum number of iterations
    alpha = 1;             % Pheromone importance weight (Eq.7)
    beta = 2;              % Heuristic importance weight (Eq.7)
    gamma = 2;             % Exploration factor importance weight (Eq.7)
    rho = 0.05;            % Pheromone evaporation rate (Eq.18)
    p0 = 0.2;              % Greedy selection threshold (Eq.8)
    b_discount = 0.6;      % Discount factor for Q-learning (Eq.14)
    lambda = 0.6;          % Learning weight for Q-value update (Eq.13)
    
    % -------- 4. Initialize pheromone, heuristic, Q-table, V --------
    pheromone = ones(rows, cols);
    pheromone(grid_1==1) = 0;
    
    heuristic = zeros(rows, cols);
    for r = 1:rows
        for c = 1:cols
            d = norm([r,c] - goalPos);
            heuristic(r,c) = 1 / (d + 0.1);  % heuristic = inverse distance
        end
    end
    heuristic(grid_1==1) = 0;
    
    stateCount = rows * cols;
    Q_table = zeros(stateCount, 8);   % Q-value for each state-action pair
    V = zeros(rows, cols);            % Node value (will be recomputed each iteration)
    
    % -------- 5. Initialize global best tracking --------
    globalBestPath = [];
    globalBestPathLength = inf;
    lastBestPath = [];
    bestHistory = zeros(maxIter, 1);
    
    tic;   % start timer

    % -------- 6. Main iteration loop --------
    for iter = 1:maxIter
        % ---- Reset per-iteration variables ----
        antPaths = cell(numAnts,1);
        antPathLengths = inf(numAnts,1);
        successCount = 0;
        V = zeros(rows, cols);
        nodeCount_iter = zeros(rows, cols);
        edgeCount_iter = zeros(rows, cols, 8);
        w = ones(rows, cols);
        w(grid_1==1) = 0;
        A_set_logical = false(rows, cols);      % nodes in successful paths
        Aprime_set_logical = false(rows, cols); % nodes in failed paths
        
        % Precompute masks for current and previous best paths
        isCurBest = false(rows, cols);
        if ~isempty(globalBestPath)
            for i = 1:size(globalBestPath,1)
                isCurBest(globalBestPath(i,1), globalBestPath(i,2)) = true;
            end
        end
        isPrevBest = false(rows, cols);
        if ~isempty(lastBestPath)
            for i = 1:size(lastBestPath,1)
                isPrevBest(lastBestPath(i,1), lastBestPath(i,2)) = true;
            end
        end

        % ---- 6a. Ant path construction ----
        for ant = 1:numAnts
            currentPos = startPos;
            path = currentPos;
            pathLength = 0;
            visited = false(rows, cols);
            visited(startPos(1), startPos(2)) = true;
            
            for step = 1:500   % max steps to prevent infinite loops
                if isequal(currentPos, goalPos), break; end
                
                % Get free neighbors not yet visited
                allNbrs = neighborMap{currentPos(1), currentPos(2)};
                valid = [];
                for i = 1:size(allNbrs,1)
                    if ~visited(allNbrs(i,1), allNbrs(i,2))
                        valid = [valid; allNbrs(i,:)];
                    end
                end
                if isempty(valid), break; end
                
                % Compute transition probabilities (Eq.7)
                probs = zeros(size(valid,1),1);
                sum_w = sum(w(sub2ind([rows,cols], valid(:,1), valid(:,2))));
                for i = 1:size(valid,1)
                    nr = valid(i,1); nc = valid(i,2);
                    if sum_w == 0
                        sigma = 0;
                    else
                        sigma = w(nr,nc) / sum_w;
                    end
                    probs(i) = (pheromone(nr,nc)^alpha) * (heuristic(nr,nc)^beta) * (sigma^gamma);
                end
                if sum(probs)==0
                    probs = ones(size(probs)) / length(probs);
                else
                    probs = probs / sum(probs);
                end
                
                % Node selection: greedy or roulette (Eq.8)
                if rand() > p0
                    [~, idx] = max(probs);
                else
                    cumprob = cumsum(probs);
                    idx = find(cumprob >= rand(), 1, 'first');
                    if isempty(idx), idx = randi(length(probs)); end
                end
                nextPos = valid(idx,:);
                
                pathLength = pathLength + norm(nextPos - currentPos);
                path = [path; nextPos];
                currentPos = nextPos;
                visited(currentPos(1), currentPos(2)) = true;
            end
            
            % Record path outcome
            if isequal(currentPos, goalPos)
                antPaths{ant} = path;
                antPathLengths(ant) = pathLength;
                successCount = successCount + 1;
                for i = 1:size(path,1)
                    A_set_logical(path(i,1), path(i,2)) = true;
                end
                
                if pathLength < globalBestPathLength
                    lastBestPath = globalBestPath;
                    globalBestPath = path;
                    globalBestPathLength = pathLength;
                end
            else
                antPaths{ant} = path;
                for i = 1:size(path,1)
                    Aprime_set_logical(path(i,1), path(i,2)) = true;
                end
            end
        end

        % ---- 6b. Update exploration auxiliary factor w (Eq.10) ----
        inA_only = A_set_logical & ~Aprime_set_logical;
        inAprime_only = Aprime_set_logical & ~A_set_logical;
        both = A_set_logical & Aprime_set_logical;
        w(inA_only) = w(inA_only) * 0.9;        % reward
        w(inAprime_only) = w(inAprime_only) * 0.3; % penalty
        w(both) = w(both) * 0.7;                % neutral
        w = max(w, 0.1);
        w(grid_1==1) = 0;

        % ---- 6c. Count node/edge frequencies for V computation ----
        for a = 1:numAnts
            if antPathLengths(a) < inf
                path = antPaths{a};
                for i = 1:size(path,1)-1
                    r = path(i,1); c = path(i,2);
                    nodeCount_iter(r,c) = nodeCount_iter(r,c) + 1;
                    nr = path(i+1,1); nc = path(i+1,2);
                    diff = [nr - r, nc - c];
                    actIdx = find(ismember(dirs, diff, 'rows'), 1);
                    if ~isempty(actIdx)
                        edgeCount_iter(r,c,actIdx) = edgeCount_iter(r,c,actIdx) + 1;
                    end
                end
            end
        end

        % ---- 6d. Q-value update (Eq.13-15) ----
        % Note: This implementation uses a suffix-distance array to compute
        % Q_est efficiently in O(L) per path (rather than O(L^2)).
        for a = 1:numAnts
            if antPathLengths(a) < inf
                path = antPaths{a};
                N = size(path,1);
                suffixDist = zeros(N,1);
                for i = N:-1:1
                    if i == N
                        suffixDist(i) = 0;
                    else
                        suffixDist(i) = suffixDist(i+1) + norm(path(i,:) - path(i+1,:));
                    end
                end
                
                for i = 1:N-1
                    s = path(i,:); s_next = path(i+1,:);
                    sidx = sub2ind([rows, cols], s(1), s(2));
                    diff = s_next - s;
                    act_idx = find(ismember(dirs, diff, 'rows'), 1);
                    if isempty(act_idx), continue; end
                    
                    % Estimate Q-value (Eq.14-15) using suffix distances
                    Q_est = 0;
                    for v = i:N
                        sub_len = suffixDist(v);
                        Q_est = Q_est + (b_discount^(v-i)) * (1/(sub_len + 1));
                    end
                    oldQ = Q_table(sidx, act_idx);
                    Q_table(sidx, act_idx) = max([oldQ, Q_est, lambda*oldQ + (1-lambda)*Q_est]); % Eq.13
                end
            end
        end

        % ---- 6e. Sparse asynchronous V update (Eq.17) ----
        visited_nodes_mask = (nodeCount_iter > 0);
        [rows_idx, cols_idx] = find(visited_nodes_mask);
        for k = 1:length(rows_idx)
            r = rows_idx(k); c = cols_idx(k);
            K = nodeCount_iter(r,c);
            if K == 0, continue; end
            sumVal = 0;
            for a = 1:8
                nr = r + dirs(a,1); nc = c + dirs(a,2);
                if nr>=1 && nr<=rows && nc>=1 && nc<=cols && grid_1(nr,nc)==0
                    N_val = edgeCount_iter(r,c,a);
                    if N_val > 0
                        sidx = sub2ind([rows, cols], r, c);
                        sumVal = sumVal + N_val * Q_table(sidx, a);
                    end
                end
            end
            V(r,c) = sumVal / K;
        end
        maxV = max(V, [], 'all');
        if maxV > 0, V = V / maxV; end

        % ---- 6f. Pheromone update (Eq.18-19) ----
        pheromone = (1 - rho) * pheromone; % evaporation
        Delta = zeros(rows, cols);
        L_opt = globalBestPathLength;
        L_prev = inf; 
        if ~isempty(lastBestPath)
            L_prev = calculate_path_length(lastBestPath);
        end
        
        % Compute differentiated pheromone increment (Eq.12)
        for a = 1:numAnts
            if antPathLengths(a) < inf
                path = antPaths{a}; L_f = antPathLengths(a);
                for i = 1:size(path,1)-1
                    r = path(i,1); c = path(i,2);
                    nr = path(i+1,1); nc = path(i+1,2);
                    on_cur = isCurBest(r,c) && isCurBest(nr,nc);
                    on_prev = isPrevBest(r,c) && isPrevBest(nr,nc);
                    
                    if on_cur
                        inc = exp(1 / L_opt);   % case (A)
                    elseif on_prev && ~on_cur
                        inc = 0.5 * sin(1/L_f) / sin(1); % case (B)
                    else
                        if L_prev == inf
                            inc = 1;
                        else
                            ratio = (L_f - L_opt) / (L_prev - L_opt + 1e-6);
                            inc = 2 / (1 + exp(ratio - 1)); % case (C)
                        end
                    end
                    Delta(r,c) = Delta(r,c) + inc;
                end
            end
        end
        
        % Add reinforcement with random perturbation and V scaling
        if max(Delta, [], 'all') > 0
            delta_norm = Delta / max(Delta, [], 'all');
            rand_factor = rand(rows, cols);
            update_mask = (grid_1 == 0);
            pheromone(update_mask) = pheromone(update_mask) + ...
                rand_factor(update_mask) .* delta_norm(update_mask) .* V(update_mask);
        end
        pheromone = max(pheromone, 0.01);
        pheromone(grid_1 == 1) = 0;

        % ---- 6g. Record history ----
        bestHistory(iter) = globalBestPathLength;
        % (Optional early termination code can be uncommented)
    end

    runTime = toc;
    
    % -------- 7. Post-process results --------
    bestPath = globalBestPath; 
    bestLen = globalBestPathLength;
    bestHistory = bestHistory(1:iter);
    convIter = find(bestHistory == bestLen, 1, 'first');
    if isempty(convIter), convIter = iter; end
end

% -------- Helper function --------
function L = calculate_path_length(path)
    L = 0;
    for i = 1:size(path,1)-1
        L = L + norm(path(i,:) - path(i+1,:));
    end
end