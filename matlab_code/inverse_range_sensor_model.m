% Inverse range sensor model as seen in Probabilistic Robotics

function l = inverse_range_sensor_model(xi, yi, pose, z, dmax)
    alpha = 0.15;
    beta = abs(z(2, 2) - z(2, 1));
    
    l_0 = 0;
    l_occ = 2;
    l_free = -2;
    
    x = pose(1);
    y = pose(2);
    th = pose(3);
    
    r = sqrt((xi - x)^2 + (yi - y)^2);
    phi = atan2(yi - y, xi - x) - th;
    phi = wrapTo2Pi(phi);
    
    [~, k] = min(abs(phi - z(2, :)));
    
    if (r > min(dmax, z(1, k) + alpha/2)) || (abs(phi - z(2, k)) > beta/2)
        l = l_0;
    elseif (z(1, k) < dmax) && (abs(r - z(1, k)) < alpha/2)
        l = l_occ;
    elseif (r <= z(1, k))
        l = l_free;
    else
        l = 0;
        disp('exception on inverse_range_sensor_model');
    end
    
end