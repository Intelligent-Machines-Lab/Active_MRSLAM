% SLAM USING MATLAB SLAM AND AUTONOMOUS EXPLORATION
% MADE FOR SCRIPT, AIMS TO GET FINAL RESULT ONLY, DOESNT PLOT WHILE RUNNING
% k is the iteration
% s is a TaskStructure object
% dt is the time step in seconds
function my_slam_6(k, s, dt, matlabSLAM)
    % Get app handles for plots and switches
%     axScan = app_st.axScan;
%     axFeat = app_st.axFeat;
%     dirty_or_clean = app_st.Dirty;
%     axOG = app_st.axOG;
%     logodds = app_st.LogOdds;
%     dt = app_st.dt;

    % Motion control
    if (s.mode == 2) || (s.mode == 4)
        % Adding first pose as first target
        if k == 2
            gpose = s.pos2grid([0; 0]);
            s.targets = cat(1, s.targets, gpose);
        end
        if k > 2
            % EXPLORATION PART <==================================================
            % If reach proximity to goal, reset path
            if ~isempty(s.cont.Waypoints)
                if norm(s.poses(1:2, end) - s.cont.Waypoints(end, :)') <= s.wp_radius
                    s.path = [];
                    disp('Reset waypoints!');
                end
            end
            
            % Exploration target and path generation
            if isempty(s.path)
                valid_path = s.update_path();
            else
                valid_path = 1;
            end
            
            
            % Controller takes out if there was a valid path 
            if valid_path
                s.check_if_stuck();
                if s.am_i_stuck
                    % Computes dt for a turn around at max w
                    disp('Going backwards! Stuck');
                    reverse_v = [-s.fixV; 0];
                    s.u = cat(2, s.u, reverse_v);
                    % Forbid the current rule1 (testing)
%                     s.update_path(s.cont.Waypoints(end, :));
                    s.update_path();
%                     s.path = [];
                else
                    if strcmp(s.active_cont, 'Global')
                        % Global controller________________
                        % Compute robot velocities from planned path (using
                        % PurePursuit)
                        [vx, wz] = s.cont(s.poses(:, end));
                        
                        
                    elseif strcmp(s.active_cont, 'Local')
                        % Local controller________________
                        % Compute robot velocities from scans and target
                        % direction (using VFH)
                        scan = lidarScan(s.hist_z{k-1}(1, :), s.hist_z{k-1}(2, :));
                        
                        % Calculate target direction
                        pose1 = s.poses(:, end);
                        pose2 = s.cont.Waypoints(end, :)';
                        R1_inv = [cos(-pose1(3)), -sin(-pose1(3)); sin(-pose1(3)), cos(-pose1(3))];
                        xy1_rel = R1_inv * (pose2(1:2) - pose1(1:2));
                        [angle_target, ~] = cart2pol(xy1_rel(1), xy1_rel(2));
%                         angle_target = ang_vec((s.cont.Waypoints(end, :) - s.poses(1:2, end)'), [cos(s.poses(3, end)), sin(s.poses(3, end))]) + s.poses(3, end);
                        s.m1 = angle_target;
                        direction = s.cont_local(scan, angle_target);
                        
                        if isnan(direction)
                            vx = 0;
                            dt = pi/s.maxW;
                            wz = s.maxW;
                            disp('Turning around! Too close to obstacle!');
                        else
                            s.m2 = direction; % For debug
                            
                            vx = s.fixV-0.1;
                            wz = s.dir2w(direction);
                        end
                    end
                    s.u = cat(2, s.u, [vx; wz]);
                end
            else
                disp('Going backwards! No valid path found');
                negative_u = -s.u(:, end);
                s.u = cat(2, s.u, negative_u);
            end
            
            % Apply last input u to robot and move
            s.move_and_stop(dt)
            
        end
    end

    % SLAM PART <==================================================
    % Read scan measurement
    if s.mode == 1
        
        [z_aux, data] = s.get_offline_data(k);
    
    elseif s.mode == 2
%         pause(1);
        z_aux = s.receive_scan();
        % Commented because I believe in my calculations! 
%         z_aux(2, :) = wrapTo2Pi(z_aux(2, :));
        [~, idx] = sort(z_aux(2, :));
        z_aux = z_aux(:, idx);
        [xAll, yAll] = pol2cart(z_aux(2, :), z_aux(1, :));
        data = double([xAll; yAll]');
    elseif s.mode == 3

        [z_aux, data] = s.get_offline_data(k);
        
    elseif s.mode == 4
        if strcmp(s.name, 'Isis')
%             pause(1);
            z_aux = s.receive_scan();

            z_aux(2, :) = wrapTo2Pi(z_aux(2, :) + pi); % + PI because the sensor is backwards
            [~, idx] = sort(z_aux(2, :));
            z_aux = z_aux(:, idx);
            
            z_aux = z_aux(:, ~isinf(z_aux(1, :))); % If inf noise is present, it becomes impossible to find target without interference
            z_aux(1, z_aux(1, :) > s.dmax) = Inf;
            
            [xAll, yAll] = pol2cart(z_aux(2, :), z_aux(1, :));
            data = double([xAll; yAll]');
        elseif strcmp(s.name, 'Omni')
            
            % This should last timeOut seconds at max
            z_aux = start_scan_omni(s.ROS_params.flag, s.ROS_params.pub_flag, s.ROS_params.laser);
            
            % Saturation
            z_aux(1, z_aux(1, :) > s.dmax) = Inf;
            
            [~, idx] = sort(z_aux(2, :));
            z_aux = z_aux(:, idx);
            z_aux(2, :) = wrapTo2Pi(z_aux(2, :));
            [xAll, yAll] = pol2cart(z_aux(2, :), z_aux(1, :));
            data = double([xAll; yAll]');
        end
    end
    % ATTENTION TO THIS ###################################################
    % Using 'INF Filter':
    %   current_data = feature ext, RANSAC, ICP
    % NOT using 'INF Filter':
    %   z_t = occupancy grid mapping
    % #####################################################################
    z_t = double(z_aux);
    
    current_data = data(~isinf(data(:, 1)), :);
%     data_th = 0.8 * size(current_data, 1);
    
    % Feature extraction
%     list = my_split_and_merge(current_data, s.eps);
    % STORING FEAT (EGO) ON TASKSTRUCTURE <======================
%     s.hist_feat{k} = list(:, 1);

    scan = lidarScan(z_t(1, :), z_t(2, :));
    
    if strcmp(s.name, 'Omni')
        [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(matlabSLAM, scan);
    else
        if ~isempty(s.u)
            odom = update_odom([0;0;0], s.u(:, end), dt);
        else
            odom = [0; 0; 0];
        end

        [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(matlabSLAM, scan, odom);
    end
%     [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(matlabSLAM, scan);

    if isScanAccepted
        if ~isempty(loopClosureInfo.Scores)
            s.score = cat(1, s.score, loopClosureInfo.Scores(1));
        end
        [~, optimizedPoses]  = scansAndPoses(matlabSLAM);
        if size(optimizedPoses, 1) < k
            all_poses = cat(2, s.poses, optimizedPoses(end, :)');
        else
            all_poses = optimizedPoses';
        end
        
    else
        disp('Scan rejected!');
        R_odom = eul2rotm([odom(3), 0, 0]); R_odom = R_odom(1:2, 1:2);
        t_odom = odom(1:2);
        odom_pose = apply_rt(s.poses(1:2, end)', R_odom, t_odom);
        odom_pose = [odom_pose, s.poses(3, end) + odom(3)]';
        
%         addRelativePose(matlabSLAM.PoseGraph, odom);
%         addScan(matlabSLAM, scan);

        all_poses = cat(2, s.poses, odom_pose); 
    end

    % STORING POSES ON TASKSTRUCTURE <======================
    s.poses = all_poses;

    % STORING RT ON TASKSTRUCTURE <======================
%         s.hist_rt{1} = cat(1, s.hist_rt{1}, [R, t, [k;k]]);

    % STORING SCANS (EGO) IN CART FORM ON TASKSTRUCTURE <======================
    s.hist_scans = cat(1, s.hist_scans, current_data);

    % STORING SCANS TRANSF IN CART FORM ON TASKSTRUCTURE <======================
    new_scan_transf = applyPoses2Scans(s.poses(:, end), s.hist_scans(end));
    if isempty(s.scans_transf)
        s.scans_transf{1} = cat(1, s.scans_transf, new_scan_transf);
    else
        s.scans_transf{1} = cat(1, s.scans_transf{1}, new_scan_transf);
    end

    % STORING FEAT MAP ON TASKSTRUCTURE <======================
%     s.feat_map = applyPoses2Feats(s.poses, s.hist_feat);

%     if ~isempty(axScan)
%         s.viz_scans(axScan, 1);
%     end
%     if ~isempty(axFeat)
%         s.viz_feat_map(axFeat, dirty_or_clean, 1);
%     end
    % STORING OG MAP ON TASKSTRUCTURE <======================
    s.og_map = occupancy_grid(s.og_map, s.poses(:, k), z_t, s.res, s.dmax);
%     if ~isempty(axOG)
%         s.viz_og_map(axOG, logodds);
%     end

    % STORING SCANS (EGO) IN POLAR FORM ON TASKSTRUCTURE <======================
    s.hist_z{k} = z_t;
    
    
    %{
    aux_icp_th = s.icp_th;
    aux_method_icp = s.method_icp;
    usedRansac = 0;
    if k==1
        last_data = current_data;
        s.hist_scans{k} = last_data;

    else
        last_data = s.hist_scans{k-1};
        nd = 0;
        repeated = 0;
        while(nd < data_th)
            if repeated > 1
                aux_icp_th = aux_icp_th / 10;
                if aux_icp_th < 1e-14
                    aux_icp_th = s.icp_th;
                    aux_method_icp = aux_method_icp + 1;
                    if aux_method_icp > 4
                        aux_method_icp = s.method_icp;

                        %disp('Going RANSAC');
                        usedRansac = 1;
                        [R, t, ~, ~, ~] = rt_RANSAC(current_data, last_data, s.params);
                        data_transf = apply_rt(current_data, R, t);
                        data_transf = data_transf';
                    end
                end
            end
            if ~usedRansac
                [R, t, data_transf] = icp(double(last_data'), double(current_data'), s.maxIterICP, s.minIterICP, aux_method_icp, aux_icp_th);
            end
            [~, D] = knnsearch(data_transf', last_data);
            nd = nnz(D < s.knn_th); % Acceptance threshold
            % Prevent teletransport...
            if (nd >= data_th) && (norm(t) > (s.fixV * dt))
                [R, t, ~, ~, ~] = rt_RANSAC(current_data, last_data, s.params);
                data_transf = apply_rt(current_data, R, t);
                data_transf = data_transf';
                [~, D] = knnsearch(data_transf', last_data);
                nd = nnz(D < s.knn_th); % Acceptance threshold
            end
            repeated = repeated + 1;
            if usedRansac && (nd < data_th)
                data_th = data_th * (0.7/0.8);
            end
            usedRansac = 0;
        end
        
    
        % STORING RT ON TASKSTRUCTURE <======================
        s.hist_rt{1} = cat(1, s.hist_rt{1}, [R, t, [k;k]]);
        last_data = current_data;
        
        % STORING SCANS (EGO) IN CART FORM ON TASKSTRUCTURE <======================
        s.hist_scans{k} = last_data;
        % STORING SCANS TRANSF IN CART FORM ON TASKSTRUCTURE <======================
        [s.scans_transf{1}, poses] = backtransf_scans(s.hist_scans(:), s.hist_rt{1});
        feat_transf = backtransf_feats(s.hist_feat(:), s.hist_rt{1});
        %s.hist_feat_transf{k} = cat(1, s.hist_feat_transf{k}, feat_transf{1:k});
        
        % Feat map update and plot--------
        % STORING FEAT MAP ON TASKSTRUCTURE <======================
        s.feat_map = feat_transf(1:k);
        % STORING POSES ON TASKSTRUCTURE <======================
        s.poses = poses(:, 1:k);
        if ~isempty(axScan)
            s.viz_scans(axScan, 1);
        end
        if ~isempty(axFeat)
            s.viz_feat_map(axFeat, dirty_or_clean, 1);
        end
        % OG map update and plot---------
        % STORING OG MAP ON TASKSTRUCTURE <======================
%         s.og_map = occupancy_grid(s.og_map, s.poses(:, k), z_t(:, z_t(1, :) <= 12), s.res, s.dmax);
        s.og_map = occupancy_grid(s.og_map, s.poses(:, k), z_t, s.res, s.dmax);
        if ~isempty(axOG)
            s.viz_og_map(axOG, logodds);
        end
        
        % STORING SCANS (EGO) IN POLAR FORM ON TASKSTRUCTURE <======================
        s.hist_z{k} = z_t;

    end
    %}
end