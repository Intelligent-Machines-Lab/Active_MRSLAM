% Greedy search algorithm
% My version
% input: map = OG map
%        start = [x, y] position in grid index reference frame
%        goal = [x, y] position in grid index reference frame
%        h_num = heuristic selector, see 'heuristic' function below
% output: path = [x1, y1; x2, y2 ...] position in grid index reference frame
%         closedMap = explored nodes in search, logical matrix
%         costMap = 3D map containing all costs [f = (:, :, 3)]

function [path, closedMap, costMap] = greedy_search(map, start, goal, h_num)
    costMap = do_cost_map(map);
    parentMap = cell(size(map));
    closedMap = zeros(size(map));
    path = [];
    
    start_f = heuristic(start, goal, h_num);
    costMap(start(1), start(2), 2) = start_f;
    
    
    pq = PriorityQueue();
    pq.push(start_f, start);
%     node_idx = pq.pop();
    j = 0;
    while(pq.size > 0)
        j = j + 1;
        if j >= (size(map, 1) * size(map, 2))*0.7
            disp('Path finding ended by overflow!');
            path = [];
            return;
        end
        
        node_idx = pq.pop();
        closedMap(node_idx(1), node_idx(2)) = 1;
        
        if (node_idx == goal)
            path = construct_path(parentMap, goal);
            return;
        end
        
        suc = get_successor(costMap(:, :, 1), node_idx(1), node_idx(2));
        
        for s=1:size(suc, 1)
            if (closedMap(suc(s, 1), suc(s, 2)) == 1)
                continue;
            end
            
%             node2suc = costMap(node_idx(1), node_idx(2), 2) + get_edge_cost(costMap(:, :, 1), node_idx, suc(s, :));
            h = heuristic(suc(s, :), goal, h_num);
            
            costMap(suc(s, 1), suc(s, 2), 2) = h;
            parentMap{suc(s, 1), suc(s, 2)} = node_idx;
            pq.push(costMap(suc(s, 1), suc(s, 2), 2), suc(s, :));
        end
    end
    
end

function successor = get_successor(costMap, i, j)
    % [a,b; c,d; ...]
    successor = [];
    
    for di=-1:1
        for dj=-1:1
            if((di~=0) || (dj~=0))
                if(((i+di) ~= 0) && ((j+dj) ~= 0) && ((i+di) <= size(costMap, 1)) && ((j+dj) <= size(costMap, 2)) && (costMap(i+di, j+dj) ~= Inf))
                    successor = cat(1, successor, [i+di, j+dj]);
                end
            end
        end
    end
end

function costMap = do_cost_map(map)
    costMap = ones(size(map));
    % Inf cost to obstacles, 1 to free cells
    costMap(map == 1) = Inf;
%     costMap = costMap';
    % h cost
    costMap = cat(3, costMap, Inf(size(map)));
    
end

function cost = heuristic(start, finish, whichOne)
    if (whichOne == 1)
        % Euclidean distance
        cost = sqrt((start(1) - finish(1)) ^ 2 + (start(2) - finish(2)) ^ 2);
    elseif (whichOne == 2)
        % Diagonal distance
        dx = abs(start(1) - finish(1));
        dy = abs(start(2) - finish(2));
        D = 1.0; D2 = 1.4142;
        cost =  D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
    elseif (whichOne == 3)
        % Weighted A* with euclidean
        cost = sqrt((start(1) - finish(1)) ^ 2 + (start(2) - finish(2)) ^ 2);
        eps = 5;
        cost = eps * cost;
    elseif (whichOne == 4)
        % Weighted A* with diagonal
        
        dx = abs(start(1) - finish(1));
        dy = abs(start(2) - finish(2));
        D = 1.0; D2 = 1.4142;
        cost =  D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
        
        eps = 5;
        cost = eps * cost;
    elseif (whichOne == 5)
    elseif (whichOne == 6)
    elseif (whichOne == 7)
        
    end
        
end

function cost = get_edge_cost(costMap, start, finish)
    diagonal = (start(1) ~= finish(1)) && (start(2) ~= finish(2));
     
    if diagonal
        factor = 1.4142;
%         factor = 1.0;
    else
        factor = 1.0;
    end
    cost = factor * (costMap(start(1), start(2)) + costMap(finish(1), finish(2))) / 2.0;
end

function path = construct_path(parentMap, goal)
    node = goal;
    path = [];
    while(~isempty(node))
        path = cat(1, path, node);
        node = parentMap{node(1), node(2)};
    end
end

function n = depth_node(parentMap, node)
    n = 0;
    while(~isempty(node))
        n = n + 1;
        node = parentMap{node(1), node(2)};
    end
end



