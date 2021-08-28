% Compute rule1 points for exploration goal
% input: skelMap = [binary image] skeletonization of current grid map
% output: dist2frontier = distances to end points computed from graph edge
%         rule1 = graph nodes filtered by rule1
function [rule1, dist2frontier] = compute_rule1(skelMap)
    [A, node, edges] = Skel2Graph3D(1 - skelMap, 0);
    rule1 = [];
    dist2frontier = [];
    
%     figure;
%     imshow(skelMap);
%     hold on;
    
    for j=1:length(node)
        nd = node(j);
        % This filters all degree 3 or more nodes
        if length(nd.conn) > 2
            % This filters the nodes neighbor to endnodes
            if nnz([node(nd.conn).ep]) >= 2
                [r, c] = ind2sub(size(skelMap), nd.idx(1));
                rule1 = cat(1, rule1, [r, c]);
                ed = edges(nd.links);
                indexOfA = ed(logical([node(nd.conn).ep]));
                dist_aux = [];
                for i=1:length(indexOfA)
                    n1 = indexOfA(i).n1;
                    n2 = indexOfA(i).n2;
                    dist_aux = cat(1, dist_aux, (A(n1, n2)));
                end
                dist2frontier = cat(1, dist2frontier, nonzeros(mean(dist_aux)));
%                 plot(nd.comy, nd.comx, 'bo', 'LineWidth', 2);
%                 for i=1:length(indexOfA)
%                     [r, c] = ind2sub(size(skelMap), indexOfA(i).point);
%                     plot(c, r, 'g.', 'LineWidth', 2);
%                 end
                
            end
        end
    end
%     hold off;
end