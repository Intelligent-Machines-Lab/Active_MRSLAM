% Convert poses relations to transformation
% Conversion is from pose1 to pose2
% input: r = scalar, distance between poses
%        p1_angle = angle between heading of 1 and direction of detection
%        p2_angle = angle between heading of 2 and direction of detection
% output: R = [2, 2] rotation matrix
%         t = [2, 1] translation vector

function [R, t, theta_rel] = relations2transf(r, p1_angle, p2_angle)
    [t_x, t_y] = pol2cart(p2_angle, r);
    t = [t_x; t_y];
    theta_rel = -(pi - p2_angle + p1_angle);
    R = eul2rotm([theta_rel, 0, 0]); R = R(1:2, 1:2);
end


