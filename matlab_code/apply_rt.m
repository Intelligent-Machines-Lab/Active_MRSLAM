% Apply rotation and translation 2D.
% data is [m,2], R is [2,2], t is [2,1]

function data_t = apply_rt(data, R, t)
    data_t = zeros(2, size(data, 1));
    for i=1:size(data, 1)
        data_t(:, i) = R * (data(i, :)') + t;
    end
    data_t = data_t';
end