% A star algorithm
% My version
% input: map = OG map
%        cost_layer = binary map with ones on cells to put a better cost
%        start = [x, y] position in grid index reference frame
%        goal = [x, y] position in grid index reference frame
%        h_num = heuristic selector, see 'heuristic' function below
% output: path = [x1, y1; x2, y2 ...] position in grid index reference frame
%         closedMap = explored nodes in search, logical matrix
%         costMap = 3D map containing all costs [f = (:, :, 3)]

function [path, closedMap, costMap] = a_star_cost_layer(map, cost_layer, start, goal, h_num)
    map = imdilate(map, strel('disk', 2));

    costMap = do_cost_map(map, cost_layer);
    parentMap = cell(size(map));
    closedMap = zeros(size(map));
    path = [];
    
    start_f = 0 + heuristic(start, goal, h_num);
    costMap(start(1), start(2), 2) = 0;
    costMap(start(1), start(2), 3) = start_f;
    
    
    pq = PriorityQueue();
    pq.push(start_f, start);
%     node_idx = pq.pop();
    
    while(pq.size > 0)
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
            
            node2suc = costMap(node_idx(1), node_idx(2), 2) + get_edge_cost(costMap(:, :, 1), node_idx, suc(s, :));
            h = heuristic(suc(s, :), goal, h_num);
            
            if(costMap(suc(s, 1), suc(s, 2), 3) > (node2suc + h))
                costMap(suc(s, 1), suc(s, 2), 2) = node2suc;
                costMap(suc(s, 1), suc(s, 2), 3) = node2suc + h;
                parentMap{suc(s, 1), suc(s, 2)} = node_idx;
                pq.push(costMap(suc(s, 1), suc(s, 2), 3), suc(s, :));
            end
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

function costMap = do_cost_map(map, good_cost)
    costMap = ones(size(map));
    costMap(good_cost == 1) = 0.5;
    % Inf cost to obstacles, 1 to free cells
    costMap(map == 1) = Inf;
%     costMap = costMap';
    % g cost
    costMap = cat(3, costMap, Inf(size(map)));
    % f cost
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



