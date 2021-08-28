% Wasnt going to do a function for this, but meh
function new_map = recover_from_log_odds(map)
    new_map = zeros(size(map));
    for m = 1:size(map, 1)
        for n = 1:size(map, 2)
            new_map(m, n) = 1 - (1/(1 + exp(map(m, n))));
        end
    end
end