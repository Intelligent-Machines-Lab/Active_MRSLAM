% Update occupancy grid map as seen in Probabilistic Robotics
% input: map = matrix mxn to hold information
%        pose = current robot pose [x, y, th]'
%        z = current robot measurement [d, phi]'
%        res = map resolution in px/m
%        dmax = maximum distance of measurement sensor
% output: map2 = matrix mxn
function map2 = occupancy_grid(map, pose, z, res, dmax)
    map2 = zeros(size(map));
    l_0 = 0;

    for i=1:size(map, 1)
        for j=1:size(map, 2)
            xi = (j - size(map, 2)/2)/res;
            yi = (i - size(map, 1)/2)/res;
            if(sqrt((xi - pose(1))^2 + (yi - pose(2))^2) < dmax)
                map2(i, j) = map(i, j) + inverse_range_sensor_model(xi, yi, pose, z, dmax) - l_0;
            else
                map2(i, j) = map(i, j);
            end
        end
    end
    
end

