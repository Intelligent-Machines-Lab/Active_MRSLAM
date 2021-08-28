% Apply 2d rotation and translation to grid maps (logodds version)
% Works by transforming index positions and copying its content
% input: map = [m, m] matrix containing binary map
%        theta = angle of rotation in radians
%        t_row = scalar in grid units, translation in rows (positive direction: UP->DOWN)
%        t_col = scalar in grid units, translation in columns (positive direction: LEFT->RIGHT)
% output: map_t = [m, m] matrix containing transformed binary map
function map_t = transform_grid_logodds(map, theta, t_row, t_col)
    [my, mx] = size(map);
    R = eul2rotm([theta, 0, 0]);
    T1 = [1, 0, mx/2; 0, 1, my/2; 0, 0, 1];
    T2 = [1, 0, -mx/2; 0, 1, -my/2; 0, 0, 1];
    t = [1, 0, t_row; 0, 1, t_col; 0, 0, 1];
    % Sequence explained:
    % T2: translate to origin
    % R: rotate input angle around origin
    % T2: translate to original position
    % t: translate input vector
    Rt = t*T1*R*T2;
    map_t = zeros(size(map));
    for c=1:mx
        for r=1:my
            if (map(c, r) ~= 0)
                % Transformation and ""interpolation""
                aux = Rt*[c; r; 1];
                aux = floor(aux);
                if((aux(1) < mx) && (aux(2) < my) && (aux(1) > 0) && (aux(2) > 0))
                    map_t(aux(1), aux(2)) = map(c, r);
                    % "Interpolation"
                    if((aux(2)+1 < my))
                        map_t(aux(1), aux(2)+1) = map(c, r);
                    end
                    if((aux(1)+1 < mx))
                        map_t(aux(1)+1, aux(2)) = map(c, r);
                    end
                    if((aux(1)+1 < mx) && (aux(2)+1 < my))
                        map_t(aux(1)+1, aux(2)+1) = map(c, r);
                    end
                end
            end
        end
    end
end