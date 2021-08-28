% Convert poses to transformation, given they are on same reference frame
% Conversion is from pose1 to pose2
% input: pose1 = [x; y; th]
%        pose2 = [x; y; th]
% output: R = [2, 2] rotation matrix
%         t = [2, 1] translation vector
%         theta_rel = angle of R

function [R, t, theta_rel] = poses2transf(pose1, pose2)
    
%     figure;
%     subplot(131);
%     drawPose(pose1, [1, 0, 0], 5, 1);
%     drawPose(pose2, [0, 0, 1], 5, 1);
%     hold(gca, 'on');
%     plot([pose1(1), pose2(1)], [pose1(2), pose2(2)], 'k--');
%     axis(gca, 'equal');
%     grid(gca, 'on');
    
    % Solve p1 relations --------------------------------------------------
    
    R1_inv = [cos(-pose1(3)), -sin(-pose1(3)); sin(-pose1(3)), cos(-pose1(3))];
    xy1_rel = R1_inv * (pose2(1:2) - pose1(1:2));
    [theta1_rel, r1_rel] = cart2pol(xy1_rel(1), xy1_rel(2));
    
    r = norm(pose1(1:2) - pose2(1:2));
%     theta_rel = angdiff(pose1(3), pose2(3));

    v1 = [0, 0];
    R_aux = eul2rotm([theta1_rel, 0, 0]); R_aux = R_aux(1:2, 1:2);
    v2 = apply_rt([r1_rel, 0], R_aux, [0; 0]);
    
%     subplot(132);
%     drawPose([v1, 0]', [1, 0, 0], 5, 1);
%     drawPose([v2, angdiff(pose1(3), pose2(3))]', [0, 0, 1], 5, 1);
%     hold(gca, 'on');
%     plot([v1(1), 0], [v1(2), 0], 'k--');
%     plot([v2(1), 0], [v2(2), 0], 'k--');
%     plot([v2(1), v1(1)], [v2(2), v1(2)], 'k--');
%     axis(gca, 'equal');
%     grid(gca, 'on');
    
    % Solve p2 relations --------------------------------------------------
    
    
    R2_inv = [cos(-pose2(3)), -sin(-pose2(3)); sin(-pose2(3)), cos(-pose2(3))];
    xy2_rel = R2_inv * (pose1(1:2) - pose2(1:2));
    [theta2_rel, r2_rel] = cart2pol(xy2_rel(1), xy2_rel(2));
    
    v1 = [0, 0];
    R_aux = eul2rotm([theta2_rel, 0, 0]); R_aux = R_aux(1:2, 1:2);
    v2 = apply_rt([r2_rel, 0], R_aux, [0; 0]);
    
%     subplot(133);
%     drawPose([v1, 0]', [0, 0, 1], 5, 1);
%     drawPose([v2, angdiff(pose2(3), pose1(3))]', [1, 0, 0], 5, 1);
%     hold(gca, 'on');
%     plot([v1(1), 0], [v1(2), 0], 'k--');
%     plot([v2(1), 0], [v2(2), 0], 'k--');
%     plot([v2(1), v1(1)], [v2(2), v1(2)], 'k--');
%     axis(gca, 'equal');
%     grid(gca, 'on');

    % Solve transformation from relations ---------------------------------
    [R, t, theta_rel] = relations2transf(r, theta1_rel, theta2_rel);
end
