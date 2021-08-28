% SLAM USING MATLAB SLAM AND AUTONOMOUS EXPLORATION
% MADE FOR APP DESIGNER
% k is the iteration
% s is a TaskStructure object
% app_st holds the plot handles or other necessary app variables
% THIS VERSION SHOWS PLOTS DURING EXECUTION!
function my_slam_5(k, s, app_st, matlabSLAM)
    % Get app handles for plots and switches
    axScan = app_st.axScan;
    axFeat = app_st.axFeat;
    dirty_or_clean = app_st.Dirty;
    axOG = app_st.axOG;
    logodds = app_st.LogOdds;
    dt = app_st.dt;

    % Motion control
    if (s.mode == 2) || (s.mode == 4)
        % Adding first pose as first target
        if k == 2
            gpose = s.pos2grid([0, 0]);
            s.targets = cat(1, s.targets, gpose);
        end
        if k >= 3
            % EXPLORATION PART <==================================================
            % If reach proximity to goal, reset path
            if ~isempty(s.cont.Waypoints)
                if norm(s.poses(1:2, end) - s.cont.Waypoints(end, :)') <= s.wp_radius
                    s.path = [];
                    disp('Reset waypoints!');
                end
            end
            
            % Exploration target management
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
                    disp('Turning around, was stuck!');
                    dt = pi/s.maxW;
                    turn_around_u = [0; s.maxW];
                    s.u = cat(2, s.u, turn_around_u);
                else
                    % Compute robot velocities
                    [vx, wz] = s.cont(s.poses(:, end));
                    s.u = cat(2, s.u, [vx; wz]);
                end
            else
                disp('Going backwards! No valid path found');
                negative_u = -s.u(:, end);
                s.u = cat(2, s.u, negative_u);
            end

            % move
            s.ROS_params.twist_msg.Linear.X = s.u(1, end);
            s.ROS_params.twist_msg.Angular.Z = s.u(2, end);       
            send(s.ROS_params.cmd, s.ROS_params.twist_msg);
            % delay
            pause(dt);
            % stop
            s.ROS_params.twist_msg.Linear.X = 0;
            s.ROS_params.twist_msg.Angular.Z = 0;
            send(s.ROS_params.cmd, s.ROS_params.twist_msg);
            
        end
    end

    % Read scan measurement
    if s.mode == 1
        
        [z_aux, data] = s.get_offline_data(k);
    
    elseif s.mode == 2
        pause(1);
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
            pause(1);
            z_aux = s.receive_scan();

            z_aux(2, :) = wrapTo2Pi(z_aux(2, :) + pi); % + PI because the sensor is backwards
            [~, idx] = sort(z_aux(2, :));
            z_aux = z_aux(:, idx);
            [xAll, yAll] = pol2cart(z_aux(2, :), z_aux(1, :));
            data = double([xAll; yAll]');
        elseif strcmp(s.name, 'Omni')
            
            % This should last timeOut seconds at max
            z_aux = start_scan_omni(s.ROS_params.flag, s.ROS_params.pub_flag, s.ROS_params.laser);
            
            [~, idx] = sort(z_aux(2, :));
            z_aux = z_aux(:, idx);
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
    list = my_split_and_merge(current_data, s.eps);
    % STORING FEAT (EGO) ON TASKSTRUCTURE <======================
    s.hist_feat{k} = list(:, 1);

    scan = lidarScan(z_t(1, :), z_t(2, :));
    if ~isempty(s.u)
        odom = update_odom([0; 0; 0], s.u(:, end), dt);
    else
        odom = [0; 0; 0];
    end
    
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(matlabSLAM, scan, odom);

    if isScanAccepted
        if ~isempty(loopClosureInfo.Scores)
            s.score = cat(1, s.score, loopClosureInfo.Scores);
        end
        [~, optimizedPoses]  = scansAndPoses(matlabSLAM);
        all_poses = optimizedPoses';
    else
        disp('Scan rejected!');
        R_odom = eul2rotm([odom(3), 0, 0]); R_odom = R_odom(1:2, 1:2);
        t_odom = odom(1:2);
        odom_pose = apply_rt(s.poses(1:2, end)', R_odom, t_odom);
        odom_pose = [odom_pose, s.poses(3, end) + odom(3)]';

        all_poses = cat(2, s.poses, odom_pose); 
    end

    % STORING POSES ON TASKSTRUCTURE <======================
    s.poses = all_poses;

    % STORING RT ON TASKSTRUCTURE <======================
%         s.hist_rt{1} = cat(1, s.hist_rt{1}, [R, t, [k;k]]);

    % STORING SCANS (EGO) IN CART FORM ON TASKSTRUCTURE <======================
    s.hist_scans{k} = current_data;

    % STORING SCANS TRANSF IN CART FORM ON TASKSTRUCTURE <======================
    s.scans_transf{1} = applyPoses2Scans(s.poses, s.hist_scans);

    % STORING FEAT MAP ON TASKSTRUCTURE <======================
    s.feat_map = applyPoses2Feats(s.poses, s.hist_feat);

    if ~isempty(axScan)
        s.viz_scans(axScan, 1);
    end
    if ~isempty(axFeat)
        s.viz_feat_map(axFeat, dirty_or_clean, 1);
    end
    % STORING OG MAP ON TASKSTRUCTURE <======================
    s.og_map = occupancy_grid(s.og_map, s.poses(:, k), z_t, s.res, s.dmax);
    if ~isempty(axOG)
        s.viz_og_map(axOG, logodds);
    end

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