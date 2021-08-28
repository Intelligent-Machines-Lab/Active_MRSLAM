% Split and Merge implementation from Wikipedia
% https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
% input: data = in the format [x1, y1; x2, y2;...]
%        eps = paramameter for distance threshold
% output: ResultList = in the format [x1, y1; x2, y2; ...]
function ResultList = split_and_merge(data, eps)
    % Find the point with the maximum distance
    dmax = 0;
    index = 0;
    n = length(data);
    for i=2:(n - 1)
        d = point_to_line(data(i, :), data(1, :), data(n, :)) ;
        if (d > dmax)
            index = i;
            dmax = d;
        end
    end
    
    ResultList = [];
    
    % If max distance is greater than epsilon, recursively simplify
    if (dmax > eps)
        % Recursive call
        recResults1 = split_and_merge(data(1:index, :), eps);
        recResults2 = split_and_merge(data(index:n, :), eps);

        % Build the result list
        ResultList = cat(1, ResultList, [recResults1(1:length(recResults1) - 1, :); recResults2(1:length(recResults2), :)]);
    else
        ResultList = cat(1, ResultList, [data(1, :); data(n, :)]);
    end
end

