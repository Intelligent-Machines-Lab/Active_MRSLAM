% Saw this on Wikipedia
% https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
function p = line_intersect(pair1, pair2)
    x1 = pair1(1, 1);
    y1 = pair1(1, 2);
    x2 = pair1(2, 1);
    y2 = pair1(2, 2);
    x3 = pair2(1, 1);
    y3 = pair2(1, 2);
    x4 = pair2(2, 1);
    y4 = pair2(2, 2);
    
    p = zeros(1, 2);
    
    quo = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    
    if(quo == 0)
        p(1, 1) = Inf;
        p(1, 2) = Inf;
    else
        p(1, 1) = ((x1*y2 - y1*x2)*(x3 -  x4) - (x3*y4 - y3*x4)*(x1 -  x2))/quo;
        p(1, 2) = ((x1*y2 - y1*x2)*(y3 -  y4) - (x3*y4 - y3*x4)*(y1 -  y2))/quo;
    end

end