function [feats_transf] = applyPoses2Feats(poses, feats)
    feats_transf = cell(size(poses, 2), 1);
    for i=1:size(poses, 2)
        t = poses(1:2, i);
        th = poses(3, i);
        R = eul2rotm([th, 0, 0]); R = R(1:2, 1:2);
        current_feats = cell(size(feats{i}, 1), 1);
        segs = feats{i};
        for j=1:size(feats{i}, 1)
            ft = apply_rt(segs{j}, R, t);
            current_feats{j} = ft;
        end
        feats_transf{i} = current_feats;
    end
end