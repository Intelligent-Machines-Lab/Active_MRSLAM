% Found on MATLAB Answers
% answers/95608-is-there-a-function-in-matlab-that-calculates-the-shortest-distance-from-a-point-to-a-line
% Input must be 3D points (vectors), just add a 0 on Z component
% Also, here: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_vector_formulation
function d = point_to_line(pt, v1, v2)
%     v1 = [v1, 0];
%     v2 = [v2, 0];
%     pt = [pt, 0];
%     
    v1(1, 3) = 0;
    v2(1, 3) = 0;
    pt(1, 3) = 0;
    
%     v1 = cat(2, v1, 0);
%     v2 = cat(2, v2, 0);
%     pt = cat(2, pt, 0);
    
    a = v1 - v2;
    b = pt - v2;
    d = norm(cross(a,b)) / norm(a);
end