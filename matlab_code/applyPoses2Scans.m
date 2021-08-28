function [scans_transf] = applyPoses2Scans(poses, scans)
    scans_transf = {};
    for i=1:size(poses, 2)
        t = poses(1:2, i);
        th = poses(3, i);
        R = eul2rotm([th, 0, 0]); R = R(1:2, 1:2);
        st = apply_rt(scans{i}, R, t);
        scans_transf = cat(1, scans_transf, st);
    end
end