% Generates frontier list from binary occupancy grid map (should have 0.5 value on unknown)
% input: og_map_bin = matrix containing map
% output: frontier_list = [n, 2] list containing map indexes of frontier

function frontier_list = compute_frontier_cells(og_map_bin)
    B = bwboundaries(og_map_bin);
    mapSize = size(og_map_bin, 1);

    % Going through all frontier cells to determine valid ones
    frontier_list = [];
    for i=1:length(B)
        idx = B{i};
        list = cell(size(idx, 1), 1);

        for j=1:size(idx, 1)
            pt = idx(j, :);
            aux_list = get_neighbor(pt, og_map_bin);

            % KING OF NESTED IFS!
            % For all frontier cells that dont have a occupied
            % neighbor, list those that have 3 or more unknown
            % cells in their neighborhood
            if (pt(1) ~= 1) && (pt(1) ~= mapSize) && (pt(2) ~= 1) && (pt(2) ~= mapSize)
                if ~nnz(aux_list(:, 1) == 1)
                    if nnz(aux_list(:, 1) == 0.5) >= 3
                        list = cat(1, list, aux_list);
                        frontier_list = cat(1, frontier_list, pt);
                    end
                end
            end
        end
    end
end