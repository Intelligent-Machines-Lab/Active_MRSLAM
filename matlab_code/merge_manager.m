% Map merge manager
% The transformation given is from r1 to r2, so r2 is the updated map
% input: ts = cell array of TaskStructure objects
%        mergeOrder = output of detect_robot_LOS
% output: success = logical array, signs if merge was successful or not in rows of mergeOrder
%         updatedMaps = {n, 4} cell array, columns 1:3 contains successful mergeOrder index, column 4 contains structure with transformed info
function [success, updatedMaps] = merge_manager(ts, mergeOrder)
    success = false(1, size(mergeOrder, 1));
    updatedMaps = cell(size(mergeOrder, 1), 4);
    
    current_k = size(ts{1}.poses, 2);
    
%     if isempty(mergedMaps)
%         mergedMapsCut = [];
%     else
%         mergedMapsCut = mergedMaps(:, 1:2);
%     end
    
    if ~isempty(mergeOrder) && (mod(size(mergeOrder, 1), 2) == 0) 
        for i=1:size(mergeOrder, 1)
            r1 = mergeOrder{i, 1};
            r2 = mergeOrder{i, 2};
            T = mergeOrder{i, 3};
            
            % NEW STUFF----------------------------------------------------
            if ~isempty(ts{r2}.merge_info) 
                merged_before = (ts{r2}.merge_info(:, 1) == r1);
                if any(merged_before)
                    if (current_k - ts{r2}.merge_info(merged_before, 2)) > 50
                        disp([num2str(current_k), ' ', num2str(ts{r2}.merge_info(merged_before, 2))]);
                        ts{r2}.merge_info(merged_before, :) = [];
                    else
                        continue;
                    end
                end
            end
            %---------------------------------------------------------
            
            %{
            if ~isempty(mergedMapsCut)
                % Managing past merges with current detections
                % Get previous merge of r1 and r2
                
                
                mergeHappened = ismember(mergedMapsCut, [r1, r2], 'rows');
                
                if any(mergeHappened)
                    % If more than one merge is recorded, erase the oldest
                    % Fix this
                    if nnz(mergeHappened) > 1
                        mergeHappened(find(mergeHappened, 1, 'last')) = false;
                        mergedMapsCut(mergeHappened, :) = [];
                        % Update mergeIndex
                        mergeHappened = ismember(mergedMapsCut, [r1, r2], 'rows');
                    end
                    % Check if has passed at least 30 iter after last merge
                    lastMerge_k = mergedMaps(mergeHappened, 3);
                    currentMerge_k = size(ts{r1}.poses, 2);
                    if (currentMerge_k - lastMerge_k) > 50 % <== Tune this
                        % Get indexes of mergedMaps to erase the history
                        mergedMapsCut(mergeHappened, :) = [];
                        disp([num2str(currentMerge_k), ' ', num2str(lastMerge_k)]);
                    else
                        % Continue to other merges without merging r1, r2
                        continue;
                    end
                end 
            end
            %}
            
            
%             m1 = binaryMap(recover_from_log_odds(ts{r1}.og_map), 0.4, 0.7);
%             m2 = binaryMap(recover_from_log_odds(ts{r2}.og_map), 0.4, 0.7);
            disp(['Merge happening between ', num2str(r1), ' and ', num2str(r2), '...']);
            
            m1 = ts{r1}.og_map;
            m2 = ts{r2}.og_map;
%             if isempty(ts{r1}.feat_map)
%                 ts{r1}.process_scans();
%                 ts{r1}.recompute_maps();
%             end
%             if isempty(ts{r2}.feat_map)
%                 ts{r2}.process_scans();
%                 ts{r2}.recompute_maps();
%             end
%             f1 = ts{r1}.feat_map;
%             f2 = ts{r2}.feat_map;
            s1 = ts{r1}.scans_transf{1};
%             s2 = ts{r2}.scans_transf{1};
            
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
            updatedMaps(i, 1:3) = mergeOrder(i, :);
%             s.feat_map = f1f2;
            s.og_map = map_fused;
            s.scans_transf = cat(1, ts{r2}.scans_transf{1}, new_scans1_transf);
            s.targets = new_targets_r2;
            s.poses = new_pose2;
            s.hist_scans = cat(1, ts{r2}.hist_scans, ts{r1}.hist_scans);
            updatedMaps{i, 4} = s;
            
            success(i) = true;
%             if isempty(mergedMaps)
%                 mergedMaps = cat(1, mergedMaps, [r1, r2]);
%             end
%             mergedMapsCut = cat(1, mergedMapsCut, [r1, r2]);
            
            % NEW STUFF: Append to a TaskStructure variable to manage past
            % merges. At this point here, the previous was already removed
%             ts{r1}.merge_info = cat(1, ts{r1}.merge_info, [r2, current_k]);
            ts{r2}.merge_info = cat(1, ts{r2}.merge_info, [r1, current_k]);
            
            %______________________________________________________________
%             % Feat viz______________________
%             figure;
%             subplot(3,4,1);
%             ts{r1}.viz_feat_map(gca, 'Dirty', 0);
%             title('Map 1');
% 
%             subplot(3,4,2);
%             ts{r2}.viz_feat_map(gca, 'Dirty', 0);
%             title('Map 2');
% 
%             subplot(3,4,3);
%             feat_viz(gca, f1_transf);
%             axis equal;
%             grid on;
%             title('Map 1 transf');
% 
%             subplot(3,4,4);
%             feat_viz(gca, f1f2);
%             axis equal;
%             grid on;
%             title('Maps merged');
%             
%             % OG viz______________________
%             subplot(3,4,5);
%             imagesc(flipud(-m1));
%             colormap gray;
%             title('Map 1');
% 
%             subplot(3,4,6);
%             imagesc(flipud(-m2));
%             colormap gray;
%             title('Map 2');
% 
%             subplot(3,4,7);
%             imagesc(-inflated_m1_t);
%             colormap gray;
%             title('Map 1 transf');
% 
%             subplot(3,4,8);
%             imagesc(flipud(-map_fused));
%             colormap gray;
%             title('Maps merged');
%             
%             % Scans viz______________________
%             subplot(3,4,9);
%             ts{r1}.viz_scans(gca, 0);
%             hold on;
%             plot(targets_r1(1, :), targets_r1(2, :), 'g+');
%             title('Map 1');
% 
%             subplot(3,4,10);
%             ts{r2}.viz_scans(gca, 0);
%             hold on;
%             targets_r2 = ts{r2}.grid2pos(ts{r2}.targets);
%             plot(targets_r2(:, 1), targets_r2(:, 2), 'g+');
%             title('Map 2');
% 
%             subplot(3,4,11);
%             scan_map = cell2mat(new_scans1_transf);
%             plot(scan_map(:, 1), scan_map(:, 2), 'r.');
%             hold on;
%             targets_plot = ts{r1}.grid2pos(targets_r1_t);
%             plot(targets_plot(:, 1), targets_plot(:, 2), 'g+');
%             axis equal;
%             grid on;
%             title('Map 1 transf');
% 
%             subplot(3,4,12);
%             scan_map = cell2mat(s.scans_transf);
%             plot(scan_map(:, 1), scan_map(:, 2), 'r.');
%             hold on;
%             new_targets_r2_plot = ts{r2}.grid2pos(new_targets_r2);
%             plot(new_targets_r2_plot(:, 1), new_targets_r2_plot(:, 2), 'g+');
%             axis equal;
%             grid on;
%             title('Maps merged');
            
        end
        forbiddenInfo = [];
        for j=find(success)
            r1 = updatedMaps{j, 1};
            r2 = updatedMaps{j, 2};
            T = updatedMaps{j, 3};
            
            % Delete path on r1 and r2
%             ts{r1}.path = [];
%             ts{r2}.path = [];
            
            % Update variables on r2
            s = updatedMaps{j, 4};
            ts{r2}.og_map = s.og_map;
%             ts{r2}.feat_map = s.feat_map;
            ts{r2}.scans_transf{1} = s.scans_transf;
            ts{r2}.targets = s.targets;
%             ts{r2}.poses = s.poses;
            ts{r2}.hist_scans = s.hist_scans;
            
            % If still exploring, generate a non conflitable goal and carry
            % on, else 
            if ts{r2}.coverage_percentage() < 0.9
                % This should avoid conflict of interest after meging maps
                if isempty(forbiddenInfo)
                    isValid = ts{r2}.update_path();
                else
                    pointToAvoid = forbiddenInfo(forbiddenInfo(:, 1) == r1, 2:3);
                    pointToAvoid_t = T * ([pointToAvoid, 1]');
                    pointToAvoid_t = pointToAvoid_t(1:2)';
                    % This updates the path variable of r2
                    isValid = ts{r2}.update_path(pointToAvoid_t);
                    % reset info for next merge duo
                    forbiddenInfo = [];
                end

                if isValid
                    fpoint = ts{r2}.cont.Waypoints(end, :);
                    forbiddenInfo = cat(1, forbiddenInfo, [r2, fpoint]);
                else
                    disp('thats a problem right there');
                end
            end
            
        end

    end
end

function feat_transf = apply_htransf_line(feat, T)
    feat_transf = cell(size(feat, 1), 1);
    for i=1:size(feat, 1)
        scan_feats = feat{i};
        pts = cell2mat(scan_feats)';
        pts = cat(1, pts, ones(1, size(pts, 2)));
        feat_transf_mat = T * pts;
        feat_transf_mat(3, :) = [];
        feat_transf{i} = mat2cell(feat_transf_mat', repmat(2, 1, size(scan_feats, 1)));
    end
end

function feat_viz(ax, feats)
    hold(ax, 'on');
    for i=1:size(feats, 1)
        scan_feat = feats{i};
        for j=1:size(scan_feat, 1)
            segment = scan_feat{j};
            plot(ax, segment(:, 1), segment(:, 2), 'rx-');
        end
    end
end