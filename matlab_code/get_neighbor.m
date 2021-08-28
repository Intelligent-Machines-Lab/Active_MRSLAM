% Get 8-conn neighbor
% output: list = [content, row, col] of neighbor cell

function list = get_neighbor(active_cell, grid)
    list = [];
    idx = [1, 0; -1, 0; 0, 1; 0 -1; 1, 1; 1, -1; -1, 1; -1, -1];

    for i=1:8
        r = active_cell(1) + idx(i, 1);
        c = active_cell(2) + idx(i, 2);
        if (r > size(grid, 1)) || (c > size(grid, 2)) || (r < 1) || (c < 1)
            continue;
        end
        list = cat(1, list, [grid(r, c), r, c]);
    end

end