% Testing autonomous exploration

clearvars; close all;

rosshutdown;

%-----Selection of task by setting variables-------
%--------------------------------------------------
env = 'house';
coverage_th = 0.98;
% coverage_th = min(0.99, 1/n + 0.25/n);
debug = 0;
%--------------------------------------------------

if strcmp(env, 'real')
    isSim = 0;
    taskmode = 4;
else
    isSim = 1;
    taskmode = 2;
end


if strcmp(env, 'house')
    % house environment_________________
    n = 1;
    K = 600;
    total_area = 400;
elseif strcmp(env, 'willow')
    % willow5 environment_________________
    n = 5;
    K = 4000;
    total_area = 2207; % Got this from: area_from_image.m
elseif strcmp(env, 'house_3')
    % house3 environment_________________
    n = 5;
    K = 4000;
    total_area = 2500;
elseif strcmp(env, 'real')
    n = 1;
    K = 700;
    % Lab Anderson: 9.63*6.33 - (~10) = 50 m2
    % Lab Digital Map1: (49.5*0.3)*(20*0.3) = 6 * 14.85 = 89.1 m2
    % Lab Digital Map2: (1.85*3.6) - (0.97*1.05/2) + (7.8 + 0.2)*6 - (0.56*0.52) = 53.8595
    % Corridor LMI: ~150 (by Buonocore's results)
%     total_area = 89.1;
%     total_area = 64; %<~measured by robot 53.8595;
    total_area = 300;
end

if debug
    plotNumOfRows = 3;
else
    plotNumOfRows = 2;
end
numOfPlots = plotNumOfRows*n;
ax = cell(1, numOfPlots);
h1 = figure;
for i=1:numOfPlots
    ax{i} = subplot(plotNumOfRows, n, i);
end

ts = cell(1, n);
app_st = cell(1, n);
names = cell(1, n);
delay = ones(1, n);



if isSim
    % SIM ROBOTS TS CREATION ----------------------------------------------
    for r=1:n
        if strcmp(env, 'house')
            ts{r} = TaskStructure(taskmode, 'Isis', r, 600, 15, n);
            ts{r}.eps = 0.03;
            ts{r}.knn_th = 0.2;
            ts{r}.params.d_th = 0.15;
            ts{r}.params.inlier_th = 0.05;
        else
    %         ts{r} = TaskStructure(taskmode, 'Isis', r, 1600, 15, n);
            ts{r} = TaskStructure(taskmode, 'Isis', r, 900, 8, n);
            ts{r}.eps = 0.03;
            ts{r}.knn_th = 0.2;
            ts{r}.params.d_th = 0.15;
            ts{r}.params.inlier_th = 0.05;
        end

        if debug
            app_st{r}.axScan = ax{r};
            app_st{r}.axFeat = ax{r+n};
            app_st{r}.Dirty = 'Dirty';
            app_st{r}.axOG = [];
            app_st{r}.LogOdds = 'LogOdds';
            app_st{r}.dt = delay;
        end
        % Name generation
        names{r} = ['Robot ', num2str(r)];
        ts{r}.total_area = total_area;
        
    end
    gt = rossubscriber('/gazebo/model_states', 'DataFormat', 'struct');
else
    % REAL ROBOTS TS CREATION ---------------------------------------------
    % Name generation
    names = {'Isis'};
    
    for r=1:length(names)
%         ts{r} = TaskStructure(taskmode, names{r}, r, 400, 15, n);
        ts{r} = TaskStructure(taskmode, names{r}, r, 2000, 10, n);
        ts{r}.total_area = total_area;
        if strcmp(names{r}, 'Isis')
            delay(r) = 1;
        else
            delay(r) = 1.5;
        end
    end
    
end
finishingOrder = [];
idleIterations = zeros(n, 1);
stuck_index = false(n, K);
coverage = zeros(1, n);
hist_cover = zeros(K, n);
mergedMaps = [];
merge_idx = [];
mergeOrder = [];

matlabSLAM = cell(1, n);
for r=1:n
    matlabSLAM{r} = lidarSLAM(20, ts{r}.dmax);
    if strcmp(env, 'house')
        matlabSLAM{r}.LoopClosureThreshold = 150;
        matlabSLAM{r}.LoopClosureSearchRadius = 10;
    elseif strcmp(env, 'real')
        
        if strcmp(ts{r}.name, 'Omni')
            matlabSLAM{r}.LoopClosureThreshold = 350;
            matlabSLAM{r}.LoopClosureSearchRadius = 20;
        else
%             matlabSLAM{r}.LoopClosureThreshold = 450;
            matlabSLAM{r}.LoopClosureThreshold = 250;
            matlabSLAM{r}.LoopClosureSearchRadius = 8;
        end
    else
%         matlabSLAM{r}.LoopClosureThreshold = 375;
%         matlabSLAM{r}.LoopClosureThreshold = 350;
%         matlabSLAM{r}.LoopClosureMaxAttempts = 2;
        matlabSLAM{r}.LoopClosureSearchRadius = 30;
%         matlabSLAM{r}.LoopClosureSearchRadius = 40;
        
        matlabSLAM{r}.LoopClosureThreshold = 400;
%         matlabSLAM{r}.LoopClosureSearchRadius = 40;
        
%         matlabSLAM{r}.LoopClosureThreshold = 450;
%         matlabSLAM{r}.LoopClosureSearchRadius = 50;
    end
end

% MAIN LOOP _______________________________________________________________
for k=1:K
    if ~ishandle(h1) || all(ismember(1:n, finishingOrder))
        disp('All robots stopped their tasks!');
        break;
    end
    
    for r=1:n
        if ~ishandle(h1)
            break;
        end
        
        disp(names{r});
        
        % Coverage and finishing order management
        coverage(r) = ts{r}.coverage_percentage();
        stuck_index(r, k) = ts{r}.am_i_stuck;
        % Stop moving if reached threshold
        if coverage(r) >= coverage_th
            if ismember(r, finishingOrder)
                idleIterations(r) = idleIterations(r) + 1;
            else
                finishingOrder = cat(2, finishingOrder, r);
            end
            % Pass turn without doing SLAM
            continue;
            % Terminate all robots
%             break;
        end
        
        
        % Checking conflit zone
        if ~isempty(mergeOrder)
            if any(r == cell2mat(mergeOrder(:, 2)))
                other_r = cell2mat(mergeOrder(r == cell2mat(mergeOrder(:, 2)), 1));
                T_or2r = mergeOrder(r == cell2mat(mergeOrder(:, 2)), 3);
                for i=1:size(other_r, 1)
                    % This function already sets active_cont to proper
                    % value, so the controller choice should regulate
                    % itself
                    switchedCont = ts{r}.check_conflict_zone(ts{other_r(i)}, T_or2r{i});
                    if switchedCont == 1
                        disp(['Robots ', mat2str([r, other_r(i)]), ' switched controllers! (global->local)']);
                    elseif switchedCont == 2
                        disp(['Robots ', mat2str([r, other_r(i)]), ' switched controllers! (local->global)']);
                    end
                end
            end
        end
        
        
        % SLAM ____________________________________________________________
        if debug
            my_slam_5(k, ts{r}, app_st{r}, matlabSLAM{r});
            ts{r}.show_path(ax{r+2*n});
        else
            my_slam_6(k, ts{r}, delay(r), matlabSLAM{r});
            
%             if (~isempty(ts{r}.score)) && ishandle(h1)
%                 plot(ax{r}, ts{r}.score);
%                 grid(ax{r}, 'on');
%             end
            if ishandle(h1)
                p3 = ax{r};
    %             cla(p3);
    %             hold(p3, 'on');
    %             ts{r}.viz_scans(p3, 2, 200);
    %             if ~isempty(ts{r}.cont.Waypoints)
    %                 path_cont = ts{r}.cont.Waypoints;
    %                 goal = path_cont(end, :);
    %                 
    %                 targets_plot = ts{r}.grid2pos(ts{r}.targets);
    %                 plot(p3, goal(1), goal(2), 'bp');
    %                 plot(p3, targets_plot(:, 1), targets_plot(:, 2), 'g+');
    %                 plot(p3, path_cont(:, 1), path_cont(:, 2), 'b:', 'LineWidth', 1.5);
    %             end
                ts{r}.viz_og_map(p3, 'LogOdds'); % Remove this if going back to old mode above
                hold(p3, 'on');
                posOnGrid = ts{r}.pos2grid(ts{r}.poses(:, end));
                plot(p3, posOnGrid(2), posOnGrid(1), 'rx');
            end
        end
        
        if isSim
            % GT POSES ________________________________________________________
            gazebo_pose = receive_gazebo_position(gt, ['robot_', num2str(r)]);
            ts{r}.gt_poses = cat(2, ts{r}.gt_poses, gazebo_pose);
        end
        
        if (k >= 2) && (n > 1)
            % DETECTING NEARBY ROBOTS _________________________________________
            mergeOrder = detect_robot_LOS(ts, isSim);

            % MERGING _________________________________________________________
            [success, updatedMaps] = merge_manager(ts, mergeOrder);
            if any(success)
                merge_idx = cat(2, merge_idx, k);

        %         mergedMaps = cat(1, mergedMaps, [cell2mat(mergeOrder(success, 1:2)), repmat(k, nnz(success), 1)]);
                disp('Merging finished!');
            end
        end
    end
    
    
    hist_cover(k, :) = coverage;
    disp(['Iteration ', num2str(k), '. Finishing order: ', mat2str(finishingOrder), '. Coverage: ', mat2str(coverage, 4), '. Coverage threshold: ', num2str(coverage_th, 4), ', ', char(datetime('now'))]);
%     if all(coverage >= coverage_th)
    if any(coverage >= coverage_th)
        disp('Attention all robots, stop activity now!');
        break;
    end
end

rosshutdown;
% Coverage plot------------------
max_cover = max(hist_cover(1:k, :) * 100, [], 'all');

figure;
hold on;
plot(hist_cover(1:k, :) * 100, 'LineWidth', 2);
plot(repmat(coverage_th * 100, 1, k), 'r--');
plot(repmat(merge_idx, 2, 1), repmat([0; max_cover], 1, length(merge_idx)), 'k--');
hold off;
ylabel('Map explored (%)');
xlabel('Iteration');
grid on;
if isempty(merge_idx)
    legend([names, 'Area_{th}'], 'Location', 'Best');
else
    legend([names, 'Area_{th}', 'Merge'], 'Location', 'Best');
end

% Activity stats plot------------------
% total_stuck = sum(stuck_index, 2);
% stats_eff = repmat(k-1, n, 1) - total_stuck - idleIterations;
% stats_eff = [stats_eff, total_stuck, idleIterations];
% 
% figure;
% bar(categorical(names), stats_eff, 'stacked');
% ylabel('Iteration');
% legend('Exploring', 'Stuck', 'Extra', 'Location', 'Best');
% Without stuck
% total_stuck = sum(stuck_index, 2);
% stats_eff = repmat(k-1, n, 1) + total_stuck - idleIterations;
% stats_eff = [stats_eff, idleIterations];
% 
% figure;
% bar(categorical(names), stats_eff, 'stacked');
% ylabel('Iteration');
% legend('Exploring', 'Idle', 'Location', 'Best');



% % Error relative to GT plot-------------------------
if isSim
    error_poses = cell(n, 1);
    for r=1:n
        pose_trans = zeros(size(ts{r}.poses));
        % Transforming poses to first GT pose frame
        R = eul2rotm([wrapToPi(ts{r}.gt_poses(3, 1)), 0, 0]); R = R(1:2, 1:2);
        t = ts{r}.gt_poses(1:2, 1);
        pose_trans(1:2, :) = apply_rt(ts{r}.poses(1:2, :)', R, t)';
        if strcmp(env, 'house') && (r == 2)
            pose_trans(3, :) = pi + angdiff(ts{r}.poses(3, :), repmat(wrapToPi(ts{r}.gt_poses(3, 1)), size(ts{r}.poses(3, :))));
        else
            pose_trans(3, :) = angdiff(repmat(wrapToPi(ts{r}.gt_poses(3, 1)), size(ts{r}.poses(3, :))), ts{r}.poses(3, :));
        end
        % Computing error
        error_poses{r}(1:2, :) = ts{r}.gt_poses(1:2, :) - pose_trans(1:2, :);
        if strcmp(env, 'house') && (r == 2)
            error_poses{r}(3, :) = angdiff(pose_trans(3, :), wrapToPi(ts{r}.gt_poses(3, :)));
        else
            error_poses{r}(3, :) = angdiff(ts{r}.gt_poses(3, :), pose_trans(3, :));
        end
        x = 1:size(ts{r}.poses, 2);

        figure;
%         subplot(1, 3, [2, 3]);
        yyaxis left;
        plot(x, error_poses{r}(1, :), 'b--', x, error_poses{r}(2, :), 'b-');
        ylabel('Position [m]');
        xlabel('Iteration');

        yyaxis right;
        plot(x, error_poses{r}(3, :));
        ylabel('Orientation [rad]');
        grid on;
        legend('Error_x', 'Error_y', 'Error_\theta', 'Location', 'north');
        title(names{r});

%         subplot(1, 3, 1);
%         drawPose(pose_trans, [1,0,0], 5, ts{r}.noseSize); 
%         hold on; 
%         drawPose(ts{r}.gt_poses, [0,0,0], 5, ts{r}.noseSize); 
%         grid on; 
%         axis image; 
%         title({'Black is Ground Truth', 'Red is SLAM (T)'});
    end
end

% Scan, feat, grid map plot---------------------
if ~debug
    for r=1:n
%         ts{r}.process_scans();
%         ts{r}.recompute_maps();

        figure;
        ax1 = subplot(121);
        ax2 = subplot(122);
%         ax3 = subplot(133);
        hold(ax1, 'on');
        ts{r}.viz_scans(ax1, 2);
%         plot(ax1, ts{r}.poses(1, stuck_index(r, 1:k)), ts{r}.poses(2, stuck_index(r, 1:k)), 'bo');
        hold(ax1, 'off');
%         ts{r}.viz_feat_map(ax2, 'Dirty', 2);
        ts{r}.viz_og_map(ax2, 'Probabilistic');
        
%         hold(ax2, 'on');
%         plot(ax2, ts{r}.targets(:, 2), ts{r}.targets(:, 1), 'r+');

    end
end

% Recover individual maps (if needed!)
% ts{1}.og_map = zeros(ts{1}.mapSize);
% ts{1}.hist_feat = {};
% ts{1}.scans_transf{1} = {};
% ts{1}.process_scans();
% 
% ts{1}.recompute_maps();
% ts{2}.og_map = zeros(ts{2}.mapSize);
% ts{2}.hist_feat = {};
% ts{2}.scans_transf{1} = {};
% ts{2}.process_scans();
% ts{2}.recompute_maps();
% 
% for r=1:n
%     figure;
%     ax1 = subplot(121);
%     ax2 = subplot(122);
%     hold(ax1, 'on');
%     ts{r}.viz_scans(ax1, 2);
%     plot(ax1, ts{r}.poses(1, stuck_index(r, 1:k)), ts{r}.poses(2, stuck_index(r, 1:k)), 'bo');
%     hold(ax1, 'off');
%     ts{r}.viz_og_map(ax2, 'Probabilistic');
% end

% Recompute coverage values by iteration (if needed!)
% for r=2:2
%     ts{r}.og_map = zeros(ts{r}.mapSize);
%     for i=1:size(ts{r}.poses, 2)
%         ts{r}.og_map = occupancy_grid(ts{r}.og_map, ts{r}.poses(:, i), ts{r}.hist_z{i}, ts{r}.res, ts{r}.dmax);
%         hist_cover(i, r) = ts{r}.coverage_percentage();
%     end
% end
% max_cover = max(hist_cover(1:k, :) * 100, [], 'all');
% 
% figure;
% hold on;
% plot(hist_cover(1:k, :) * 100, 'LineWidth', 2);
% plot(repmat(coverage_th * 100, 1, k), 'r--');
% plot(repmat(merge_idx, 2, 1), repmat([0; max_cover], 1, length(merge_idx)), 'k--');
% hold off;
% ylabel('Map explored (%)');
% xlabel('Iteration');
% grid on;
% if isempty(merge_idx)
%     legend([names, 'Area_{th}'], 'Location', 'Best');
% else
%     legend([names, 'Area_{th}', 'Merge'], 'Location', 'Best');
% end
%--------------------------------------------------------------------------

                % Handling merging of maps already finished
%                 if (length(finishingOrder) > 1) && isempty(merged_maps) % -- First merge with the 2 that finished first
%                     [PreviousMerge, feat_result] = merge_feat_maps('m1->m2', finishingOrder', ts, PreviousMerge);
%                     merged_maps = finishingOrder;
%                 elseif (length(finishingOrder) > length(merged_maps)) && ~isempty(merged_maps) % -- Merges with a PreviousMerge nonempty
%                     [PreviousMerge, feat_result] = merge_feat_maps('m1->m2', [finishingOrder(end); 0], ts, PreviousMerge);
%                     merged_maps = finishingOrder;
%                 end

