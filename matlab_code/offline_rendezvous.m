% Offline redezvous merging

% load('real_corridor_isis_complete_2_rel', 'rel', 'ts');
load('real_corridor_isis_complete_4_rel', 'rel', 'ts');

ts1 = copy(ts{1});

figure;
ax1 = subplot(211);
ax2 = subplot(212);
hold(ax1, 'on');
ts1.viz_scans(ax1, 2);
hold(ax1, 'off');
ts1.viz_og_map(ax2, 'Probabilistic');
axis(ax2, [950, 1450, 940, 1040]);

rel1 = rel;

% load('real_corridor_isis_complete_3_rel', 'rel', 'ts');
load('real_corridor_isis_complete_5_rel', 'rel', 'ts');

ts2 = copy(ts{1});

figure;
ax1 = subplot(121);
ax2 = subplot(122);
hold(ax1, 'on');
ts2.viz_scans(ax1, 2);
hold(ax1, 'off');
ts2.viz_og_map(ax2, 'Probabilistic');
axis(ax2, [900, 1050, 950, 1400]);

rel2 = rel;

proximityOrder = [];

% Relations 2 transformation
pose1 = ts1.poses(:, end);
pose2 = ts2.poses(:, end);

% Try to get more information for r as possible, then apply
% some sort of averaging and/or outlier removal to improve
% measurement. r should be an array of measurements to be
% averaged r = [from tag1, from tag2, from RGBD1, from RGBD2, ...]
r = mean([rel1(1), rel2(1)]);
theta2_rel = rel2(2);
theta1_rel = rel1(2);

% Get transformation from relations
[Rr, tr, ~] = relations2transf(r, theta1_rel, theta2_rel);


% Robot1 transformation using (R*p + t) form
R1 = eul2rotm([pose1(3), 0, 0]); R1 = R1(1:2, 1:2);
t1 = pose1(1:2);
% Robot2 transformation using (R*p + t) form
R2 = eul2rotm([pose2(3), 0, 0]); R2 = R2(1:2, 1:2);
t2 = pose2(1:2);

% Homogeneous coordinates form
Tr = cat(1, [Rr, tr], [0, 0, 1]);
T1 = cat(1, [R1, t1], [0, 0, 1]);
T2 = cat(1, [R2, t2], [0, 0, 1]);
T = T2 * Tr / T1;

% Output which robots and transformation
proximityOrder = cat(1, proximityOrder, {1, 2, T});
proximityOrder = cat(1, proximityOrder, {2, 1, inv(T)});


% Function below in code
ss = process_merge(proximityOrder, {ts1, ts2});


% Plot
figure;
ax1 = subplot(121);
ax2 = subplot(122);
hold(ax1, 'on');
ss{2}.viz_scans(ax1, 2);
hold(ax1, 'off');
ss{2}.viz_og_map(ax2, 'Probabilistic');
axis(ax2, [800, 1050, 950, 1750]);










function ts = process_merge(mergeOrder, ts)
    for i=1:size(mergeOrder, 1)
        r1 = mergeOrder{i, 1};
        r2 = mergeOrder{i, 2};
        T = mergeOrder{i, 3};
         m1 = ts{r1}.og_map;
        m2 = ts{r2}.og_map;
        s1 = ts{r1}.scans_transf{1};
        % Extract angle and translation vector
        theta = rotm2eul(T);
        theta = theta(1);
        t = T(1:2, 3) * ts{r1}.res;

        % SCANS MERGE _________________________________________________
        new_scans1_transf = {};
        for j=1:size(s1, 1)
            points = cat(1, s1{j}', ones(1, size(s1{j}, 1)));
            points_t = (T*points)'; 
            points_t(:, 3) = [];
            new_scans1_transf = cat(1, new_scans1_transf, points_t);
        end

        % FEAT MERGE __________________________________________________
        %             f1_transf = apply_htransf_line(f1, T);
        % TODO: Fuse feats
        %             f1f2 = [f1_transf; f2];

        % GRID MERGE __________________________________________________
        % Testing logodds version
        m1_t = transform_grid_logodds(flipud(m1), theta, -t(2), t(1));
        % Do a obstacle inflation here, the problem is, I'm fusing
        % logodds maps. How can I recover a logodds from a binary?
        if isempty(ts{r1}.merge_info)
            inflated_m1_t = imdilate(m1_t, strel('disk', 1));
        else
            inflated_m1_t = m1_t;
        end
        map_fused = fuse_grid_logodds(flipud(inflated_m1_t), m2);

        % TARGETS MERGE _______________________________________________
        targets_r1 = [ts{r1}.grid2pos(ts{r1}.targets), ones(size(ts{r1}.targets, 1), 1)]';
        targets_r1_t = T * targets_r1;
        targets_r1_t = ts{r1}.pos2grid(targets_r1_t(1:2, :));
        new_targets_r2 = cat(1, ts{r2}.targets, targets_r1_t);

        % POSE MERGE __________________________________________________
        pose1 = ts{r1}.poses;
        pose1_xy_t = T * cat(1, pose1(1:2, :), ones(1, size(pose1, 2)));
        pose1_theta_t = angdiff(pose1(3, :),repmat(theta, 1, size(pose1, 2)));
        pose1_t = cat(1, pose1_xy_t(1:2, :), pose1_theta_t);
        new_pose2 = cat(2, ts{r2}.poses, pose1_t);

        % Update output variables _____________________________________
        ts{r2}.og_map = map_fused;
        ts{r2}.scans_transf{1} = cat(1, ts{r2}.scans_transf{1}, new_scans1_transf);
        ts{r2}.targets = new_targets_r2;
    %     s.poses = new_pose2;
        ts{r2}.hist_scans = cat(1, ts{r2}.hist_scans, ts{r1}.hist_scans);

    end
end

