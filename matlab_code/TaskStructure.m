% Handles all variables and basic functions associated with the SLAM task

classdef TaskStructure < matlab.mixin.SetGet & matlab.mixin.Copyable
    properties
        % Decisive vars---------
        name
        mode
        exp
        n_r
        % Split and merge-----
        eps
        % ICP------------------
        maxIterICP
        minIterICP
        icp_th
        method_icp
        knn_th
        % RANSAC, map cleaning, fuse line buonocore, check points inline
        params
        % Hist------------
        hist_scans
        hist_rt
        scans_transf
        hist_feat
        %hist_feat_transf
        hist_z
        % Maps-----------
        feat_map
        og_map
        clean_map
        bin_map
        % ROS------------
        ROS_params
        % Merging--------
        m1 = []
        m2 = []
        merge_info
        relative_info
        % Robot poses----
        poses
        gt_poses
        % Controllers----
        cont
        cont_local
        lookAhead
        maxW
        fixV
        u
        active_cont
        % Exploration----
        total_area
        targets
        path
        wp_radius
        am_i_stuck
        r_collision
        % Misc.----------
        noseSize
        env_img
        dataset
        nposes
        mapSize
        res
        dmax
        score = []
        
    end
    
    methods
        function obj = TaskStructure(mode_selection, bot_name, experiment, map_size, resolution, n_robots)
            obj.mode = mode_selection;
            obj.exp = experiment;
            obj.name = bot_name;
            obj.n_r = n_robots;
            if obj.mode == 1
                if experiment > 0
                    filename = 'dataset_gazebo_3.mat';
                    load(filename, 'hist_laser_xy', 'hist_laser', 'u');
                    obj.dataset.hist_laser_xy = hist_laser_xy;
                    obj.dataset.hist_laser = hist_laser;
                    obj.nposes = length(u);
                    obj.env_img = 'map_gazebo_1.jpg';
                    
                    obj.eps = 0.2;
                    obj.knn_th = 0.2;
                    obj.params.d_th = 0.15;
                    obj.params.inlier_th = 0.05;
                else
                    if experiment == 0
                        filename = 'dataset_gazebo_0.mat';
                    elseif experiment == -1
                        filename = 'dataset_gazebo_01.mat';
                    elseif experiment == -2
                        filename = 'willow_exp1_r1.mat';
                    end
                    load(filename, 'hist_z', 'hist_gt');
                    if exist('hist_gt', 'var')
                        obj.gt_poses = hist_gt;
                        
                    end
                    scans =  hist_z(~cellfun(@isempty, hist_z));
                    obj.dataset = scans;
                    obj.nposes = size(scans, 1);
                    obj.env_img = 'willowgarage.png';
                    
                    obj.eps = 0.1;
                    obj.knn_th = 0.2;
                    obj.params.d_th = 0.05;
                    obj.params.inlier_th = 0.02;
                end
                
                obj.dmax = 10;
                
                obj.noseSize = 0.5;
                
                % ICP stuff
                obj.maxIterICP = 10000;
                obj.minIterICP = 10;
                obj.icp_th = 1e-7;
                obj.method_icp = 1;
                

                % RANSAC stuff
                obj.params.maxIter = 500;
                obj.params.inlier_ratio = 0.975;

                % Chunk of params
                obj.params.r1_tol = pi/18;
                obj.params.r4_tol = 1;
                obj.params.d_min = 0.3;
                obj.params.d_ort = 0.3;
                obj.params.r1_tol_2 = pi/9; % 20 deg
                obj.params.little = 0.01;
                obj.params.isVertical = 0.02;
                obj.params.isHorizontal = 0.02;
                obj.params.dth = 0.05;
                obj.params.cmax = 0;

                
            elseif obj.mode == 2
                % Doing this to not cast multiple instances of rosinit
                try
                    [~] = rostopic('list');
                catch
                    %------------------------
                    [~, str] = system('hostname -I');
                    IP = str(1:strfind(str,' ')-1);
                    rosinit(IP);
                end
                
                if experiment
                    % Creating ROS stuff for multi robot spawn
                    obj.ROS_params.laser = rossubscriber(['/scan_', num2str(experiment)], 'DataFormat', 'struct');
                    [obj.ROS_params.cmd, obj.ROS_params.twist_msg] = rospublisher(['/robot_', num2str(experiment), '/cmd_vel'], 'geometry_msgs/Twist', 'DataFormat', 'struct');
                    obj.ROS_params.cmd_rec = rossubscriber(['/robot_', num2str(experiment), '/cmd_vel'], 'DataFormat', 'struct');
                    % NEW camera stuff
%                     obj.ROS_params.camera_info = rossubscriber(['/robot_', num2str(experiment), '/camera/rgb/camera_info'], 'DataFormat', 'struct');
%                     obj.ROS_params.camera_rgb = rossubscriber(['/robot_', num2str(experiment), '/camera/rgb/image_raw'], 'DataFormat', 'struct');
                else
                    % Creating ROS stuff for isis pkg
                    obj.ROS_params.laser = rossubscriber('/scan', 'DataFormat', 'struct');
                    [obj.ROS_params.cmd, obj.ROS_params.twist_msg] = rospublisher('/cmd_vel', 'geometry_msgs/Twist', 'DataFormat', 'struct');
                    obj.ROS_params.cmd_rec = rossubscriber('/cmd_vel', 'DataFormat', 'struct');
                    
                    
                end
                obj.dmax = 10;
                obj.nposes = 1e2; % big number to limit memory occupied by hist variables
                obj.eps = 0.2;
                obj.noseSize = 0.5;

                % ICP stuff
                obj.maxIterICP = 10000;
                obj.minIterICP = 10;
                obj.icp_th = 1e-7;
                obj.method_icp = 1;
                obj.knn_th = 0.2;

                % RANSAC stuff
                obj.params.maxIter = 500;
                obj.params.d_th = 0.15;
                obj.params.inlier_th = 0.05;
                obj.params.inlier_ratio = 0.75;

                % Chunk of params
                obj.params.r1_tol = pi/18;
                obj.params.r4_tol = 1;
                obj.params.d_min = 0.3;
                obj.params.d_ort = 0.3;
                obj.params.r1_tol_2 = pi/9; % 20 deg
                obj.params.little = 0.01;
                obj.params.isVertical = 0.02;
                obj.params.isHorizontal = 0.02;
                obj.params.dth = 0.05;
                obj.params.cmax = 0;

                obj.env_img = 'map_gazebo_1.jpg';
            elseif obj.mode == 3
                if strcmp(bot_name, 'Isis')
                    % Valid, old stuff: 1
                    % Valid, not on polar for some reason, cylinders!: 2, 3
                    % Valid: 4, 5, 6, 7
                    filename = ['dataset_real_isis_', num2str(experiment), '.mat'];
                    
                    if experiment == 1
                        load(filename, 'hist_lidar_isis', 'jmax', 'n_samples');
                        obj.env_img = 'map_lmi.jpg';

                        all_measurements = hist_lidar_isis(:, 1);
                        all_measurements{1} = double(all_measurements{1});

                        obj.dataset = all_measurements(1:jmax:end);
                        obj.nposes = n_samples;
                    elseif (experiment == 2) || (experiment == 3)
                        load(filename, 'hist_scans');
                        obj.env_img = 'map_ele_corridor_1.jpg';
                        
                        scans = {};
                        for i=1:size(hist_scans, 1)
                            notz = hist_scans{i};
                            if ~isempty(notz)
                                [phi, d] = cart2pol(notz(:, 1), notz(:, 2));
                                scans = cat(1, scans, [d, phi]');
                            end
                        end
                        obj.dataset = scans;
                        obj.nposes = size(scans, 1);
                    elseif experiment > 3
                        load(filename, 'hist_z');
                        obj.env_img = 'map_ele_corridor_1.jpg';

                        scans =  hist_z(~cellfun(@isempty, hist_z));

                        obj.dataset = scans;
                        obj.nposes = size(scans, 1);
                    else
                        disp('Not a valid experiment!');
                    end
                    
                    % ISIS specific vars-----
                    obj.dmax = 8; % 12
                    obj.eps = 0.05;
                    obj.params.inlier_th = 0.1;
                    obj.params.d_th = 0.15;
                    obj.icp_th = 1e-7;
                elseif strcmp(bot_name, 'Omni')
                    % Invalid: 1, 2
                    % Valid, too many points, still using the r mechanic: 3
                    % Valid, too many points: 4, 5, 6
                    % Valid: 7, 8
                    % Valid, correct height: 9, 10, 11, 12, 13
                    filename = ['dataset_real_omni_', num2str(experiment), '.mat'];
                    
                    if (experiment >= 3)
                        load(filename, 'hist_z');
                        obj.env_img = 'map_ele_corridor_1.jpg';

                        scans =  hist_z(~cellfun(@isempty, hist_z));

                        obj.dataset = scans;
                        obj.nposes = size(scans, 1);
                    else
                        disp('Not a valid experiment!');
                    end
                    % OMNI specific vars-----
                    obj.dmax = 3; %?
                    obj.eps = 0.018;
                    obj.params.inlier_th = 0.01;
                    obj.params.d_th = 0.01;
                    obj.icp_th = 1e-7;
                end
                
                if experiment < 0
                    scans = read_dataset_intel(2);
                    
                    obj.env_img = 'intel_lab.png';
                    obj.dataset = scans;
%                     obj.dataset = scans(270:3:end);
%                     obj.dataset = scans(270:end);
                    obj.nposes = size(obj.dataset, 1);
                    obj.dmax = 80;
                    obj.icp_th = 1e-9;
                    obj.params.inlier_th = 0.01;
                end
                
                obj.noseSize = 0.2;
                
                % ICP stuff
                obj.maxIterICP = 10000;
                obj.minIterICP = 10;
                
                obj.method_icp = 1;
                obj.knn_th = 0.1;
                % RANSAC stuff
                obj.params.maxIter = 500;
                
                
                obj.params.inlier_ratio = 0.975;

                % Chunk of params
                % map cleaning, my method------
                obj.params.r1_tol = pi/18;
                obj.params.r4_tol = 1;
                % map cleaning, fuse line buonocore-------
                obj.params.d_min = 0.1;
                obj.params.d_ort = 0.1;
                % obj.params.r1_tol_2 = pi/18; % 10 deg
                obj.params.r1_tol_2 = pi/9; % 20 deg
                %obj.params.r1_tol_2 = pi/6; % 30 deg
                % obj.params.r1_tol_2 = pi/4; % 45 deg
                % fuse line buonocore, check points inline-----------
                obj.params.little = 0.2;
                obj.params.isVertical = 0.4;
                obj.params.isHorizontal = 0.4;
                obj.params.dth = 0.05;
                obj.params.cmax = 0;

            elseif obj.mode == 4
%                 rosshutdown;
                %------------------------
                if strcmp(bot_name, 'Isis')
%                     IP = '192.168.0.90';
%                     rosinit(IP);
                    % Doing this to not cast multiple instances of rosinit
                    try
                        [~] = rostopic('list');
                    catch
                        %------------------------
                        [~, str] = system('hostname -I');
                        IP = str(1:strfind(str,' ')-1);
                        rosinit(IP);
                    end

                    % Creating ROS stuff
                    obj.ROS_params.laser = rossubscriber('/scan', 'DataFormat', 'struct');
                    [obj.ROS_params.cmd, obj.ROS_params.twist_msg] = rospublisher('/cmd_vel_isis', 'geometry_msgs/Twist', 'DataFormat', 'struct');
                    obj.ROS_params.cmd_rec = rossubscriber('/cmd_vel_isis', 'DataFormat', 'struct');
                    % NEW CAMERA STUFF!!!
                    obj.ROS_params.rel_info = rossubscriber('/rel_info_isis', 'std_msgs/Float64MultiArray', 'DataFormat', 'struct');
%                     obj.ROS_params.camera_info = rossubscriber('/rgb/camera_info', 'DataFormat', 'struct');
%                     obj.ROS_params.camera_rgb = rossubscriber('/rgb/image_raw', 'DataFormat', 'struct');
%                     obj.ROS_params.camera_depth = rossubscriber('/depth_registered/image_raw', 'DataFormat', 'struct');
                    % ISIS specific vars-----
                    obj.dmax = 6;
                    obj.eps = 0.05;
                    obj.params.inlier_th = 0.1;
                elseif strcmp(bot_name, 'Omni')
%                     IP = '192.168.0.111';
%                     rosinit(IP);
                    % Doing this to not cast multiple instances of rosinit
                    try
                        [~] = rostopic('list');
                    catch
                        %------------------------
                        [~, str] = system('hostname -I');
                        IP = str(1:strfind(str,' ')-1);
                        rosinit(IP);
                    end

                    % Creating ROS stuff
                    obj.ROS_params.laser = rossubscriber('/depth_scan', 'std_msgs/Float64MultiArray', 'DataFormat', 'struct');
                    [obj.ROS_params.cmd, obj.ROS_params.twist_msg] = rospublisher('/cmd_vel_omni', 'geometry_msgs/Twist', 'DataFormat', 'struct');
                    [obj.ROS_params.pub_flag, obj.ROS_params.flag] = rospublisher('/do_scan', 'std_msgs/Bool', 'DataFormat', 'struct');
                    
                    % NEW CAMERA STUFF!!!
                    obj.ROS_params.rel_info = rossubscriber('/rel_info_omni', 'std_msgs/Float64MultiArray', 'DataFormat', 'struct');
                    
                    % OMNI specific vars-----
                    obj.dmax = 3; %?
                    obj.eps = 0.15;
                    obj.params.inlier_th = 0.2;
                end
                
                obj.noseSize = 0.2;
                obj.nposes = 1e2; % big number to limit memory occupied by hist variables
                
                % ICP stuff
                obj.maxIterICP = 10000;
                obj.minIterICP = 10;
                obj.icp_th = 1e-7;
                obj.method_icp = 1;
                obj.knn_th = 0.1;
                
                % RANSAC stuff
                obj.params.maxIter = 700;
                obj.params.d_th = 0.1;
                obj.params.inlier_ratio = 0.975;

                % Chunk of params
                % map cleaning, my method------
                obj.params.r1_tol = pi/18;
                obj.params.r4_tol = 1;
                % map cleaning, fuse line buonocore-------
                obj.params.d_min = 0.01;
                obj.params.d_ort = 0.01;
                % params.r1_tol_2 = pi/18; % 10 deg
                obj.params.r1_tol_2 = pi/9; % 20 deg
                % params.r1_tol_2 = pi/6; % 30 deg
                % params.r1_tol_2 = pi/4; % 45 deg
                % fuse line buonocore, check points inline-----------
                obj.params.little = 0.2;
                obj.params.isVertical = 0.15;
                obj.params.isHorizontal = 0.15;
                obj.params.dth = 0.25;
                obj.params.cmax = 0;

                obj.env_img = 'map_ele_corridor_1.jpg';
            end
            
            obj.mapSize = map_size;
            obj.res = resolution;
            
            obj.og_map = zeros(obj.mapSize);
            obj.hist_scans = {};
            obj.hist_rt = cell(1, 1);
%             obj.scans_transf = cell(1, 1);
            obj.scans_transf = {};
            obj.hist_feat = cell(obj.nposes, 1);
            %obj.hist_feat_transf = cell(obj.nposes, 1);
            obj.hist_z = cell(obj.nposes, 1);
            
            if obj.mode == 2
                obj.fixV = 0.3;
    %             obj.fixV = 0.6;
                obj.maxW = 1.5;
                obj.wp_radius = 1.0;
                obj.lookAhead = 1.0;
            else
                if strcmp(bot_name, 'Omni')
                    obj.fixV = 0.2; % Try not to change this
                    obj.maxW = 1.0; % and this
                    obj.wp_radius = 1.0;
                    obj.lookAhead = 0.5;
                else
                    obj.fixV = 1.0;
                    obj.maxW = 1.0;
                    obj.wp_radius = 1.5;
%                     obj.lookAhead = 0.5;
                    obj.lookAhead = 1.0;
                end
            end
            
            
            obj.cont = controllerPurePursuit('DesiredLinearVelocity', obj.fixV, 'MaxAngularVelocity', obj.maxW, 'LookaheadDistance', obj.lookAhead);
            obj.am_i_stuck = 0;
            obj.r_collision = 1.5;
            
            obj.cont_local = controllerVFH('UseLidarScan', true, 'DistanceLimits', [0.5, 2], 'RobotRadius', 0.13, 'MinTurningRadius', 0.2, 'NumAngularSectors', 360, 'HistogramThresholds', [1, 2]);
            obj.active_cont = 'Global';
        end
        %------------------------------------------------------------------
        % METHODS ---------------------------------------------------------
        %------------------------------------------------------------------
        function z = receive_scan(obj)
            topic = obj.ROS_params.laser;
            lidar = receive(topic);
            z_aux(1, :) = lidar.Ranges';
            z_aux(2, :) = lidar.AngleMin:lidar.AngleIncrement:lidar.AngleMax;
            z = z_aux;
            % Output is raw!
        end
        %------------------------------------------------------------------
        function [z_aux, data] = get_offline_data(obj, k)
            
            if obj.mode == 1
                if obj.exp > 0
                    data = obj.dataset.hist_laser_xy{k, obj.exp};
                    z_aux = obj.dataset.hist_laser{k, obj.exp};
                else
                    z_aux = double(obj.dataset{k});
                    % Clipping at dmax, do i need in simulated? SEE THIS <---------
%                     z_aux = z_aux(:, z_aux(1, :) < obj.dmax);

                    % Need to rearrange and wrap to 2pi, only works well this way!
%                     z_aux(2, :) = wrapTo2Pi(z_aux(2, :));
                    [~, idx] = sort(z_aux(2, :));
                    z_aux = z_aux(:, idx);
                    [xAll, yAll] = pol2cart(z_aux(2, :), z_aux(1, :));
                    data = [xAll; yAll]';
                end
            elseif obj.mode == 3
                if obj.exp == 1
                    z_aux = obj.dataset{k}(:, ~isinf(obj.dataset{k}(1,:))) - [0; pi/2]; % Investigate this pi/2...
                else
                    z_aux = double(obj.dataset{k});
                end
                
                % NEW #####################################################
                
                % Clipping at dmax
                z_aux = z_aux(:, z_aux(1, :) < obj.dmax);
                if obj.exp > 0
                % Need to rearrange and wrap to 2pi, only works well this way!
                    z_aux(2, :) = wrapTo2Pi(z_aux(2, :));

                    [~, idx] = sort(z_aux(2, :));
                    z_aux = z_aux(:, idx);
                else
                end
                [xAll, yAll] = pol2cart(z_aux(2, :), z_aux(1, :));
                data = [xAll; yAll]';
            end
            
        end
        %------------------------------------------------------------------
        function obj = viz_og_map(obj, ax, version)
            cla(ax);
            if strcmp(version, 'Probabilistic')
                og_map_prob = recover_from_log_odds(obj.og_map);
                imagesc(ax, flipud(1 - og_map_prob));
                colormap(ax, 'gray');
            elseif strcmp(version, 'Binary')
                og_map_prob = recover_from_log_odds(obj.og_map);
                og_map_bin = binaryMap(og_map_prob, 0.4, 0.7);
                imagesc(ax, flipud(1 - og_map_bin));
                colormap(ax, 'gray');
                obj.bin_map = og_map_bin;
            else
                imagesc(ax, flipud(obj.og_map));
                colormap(ax, 'gray');
            end
            if ~strcmp(version, 'Binary')
                title(ax, 'Occupancy grid map');
%                 xlabel(ax, ['X [px / ', num2str(obj.res), ']']);
%                 ylabel(ax, ['Y [px / ', num2str(obj.res), ']']);
%                 colorbar(ax);
                xlabel(ax, ['X (grid)']);
                ylabel(ax, ['Y (grid)']);
            end
            axis(ax, 'image');
%             axis(ax, [0, obj.mapSize, 0, obj.mapSize]);
        end
        %------------------------------------------------------------------
        function obj = viz_feat_map(obj, ax, version, showPoses)
            feat_transf = obj.feat_map;
            % feats ----------
            cla(ax);
            if strcmp(version, 'Dirty')
                hold(ax, 'on');
                for i=1:size(feat_transf, 1)
                    segments = feat_transf{i};
                    for j=1:size(segments, 1)
                        segment = segments{j};
                        plot(ax, segment(:, 1), segment(:, 2), 'rx-');
                    end
                end
            else
                if isempty(obj.clean_map)
%                     map_cleaned = map_cleaning(feat_transf, 1, obj.params);
                    map_cleaned = map_cleaning(feat_transf, 0, obj.params);
                    obj.clean_map = map_cleaned;
                end
                hold(ax, 'on');
                for i=1:size(obj.clean_map, 1)
                    segment = obj.clean_map{i};
                    plot(ax, segment(:, 1), segment(:, 2), 'rx-');
                end
            end
            % poses ----------
            if showPoses == 1
                plot(ax, obj.poses(1, end:-1:1), obj.poses(2, end:-1:1), 'b:');
                for i=1:size(obj.poses, 2)
                    pose = obj.poses(:, i);
                    hold(ax, 'on');
                    plot(ax, pose(1), pose(2), 'ko');
                    [nx, ny] = pol2cart(pose(3), obj.noseSize);
                    plot(ax, [pose(1), nx + pose(1)], [pose(2), ny + pose(2)], 'k-', 'LineWidth', 1.5);
                end
            elseif showPoses == 2
                plot(ax, obj.poses(1, end:-1:1), obj.poses(2, end:-1:1), 'k-');
            end
            
            axis(ax, 'image');
            grid(ax, 'on');
        end
        %------------------------------------------------------------------
        function viz_scans(obj, ax, showPoses, howMany)
            % poses ----------
            if showPoses == 1
                plot(ax, obj.poses(1, end:-1:1), obj.poses(2, end:-1:1), 'b:');
                for i=1:size(obj.poses, 2)
                    pose = obj.poses(:, i);
                    hold(ax, 'on');
                    plot(ax, pose(1), pose(2), 'ko');
                    [nx, ny] = pol2cart(pose(3), obj.noseSize);
                    plot(ax, [pose(1), nx + pose(1)], [pose(2), ny + pose(2)], 'k-', 'LineWidth', 1.5);
                end
            elseif showPoses == 2
                plot(ax, obj.poses(1, end:-1:1), obj.poses(2, end:-1:1), 'k-'); 
            end
            hold(ax, 'on');
            if (nargin > 3) && (length(obj.scans_transf{1}) > howMany)
                aux = obj.scans_transf{1};
                scan_map = cell2mat(aux(end-howMany:end));
            else
                scan_map = cell2mat(obj.scans_transf{1});
            end
            
            % scans ----------
            plot(ax, scan_map(:, 1), scan_map(:, 2), 'r.');
            axis(ax, 'image');
            grid(ax, 'on');
            title(ax, '2D Point cloud');
            xlabel(ax, 'X (m)');
            ylabel(ax, 'Y (m)');
        end
        %------------------------------------------------------------------
        function obj = set_maps(obj, whichOne)
%             final_maps = {og_map_prob, map_cleaned, og_map_bin};
            final_maps = {obj.og_map, obj.clean_map, obj.bin_map};
            if whichOne == 1
                obj.m1 = final_maps;
            else
                obj.m2 = final_maps;
            end
        end
        %------------------------------------------------------------------
        function save_scans(obj, filename)
            hist_scans = obj.hist_scans; %#ok<PROPLC>
            hist_z = obj.hist_z; %#ok<PROPLC>
            
            save(filename, "hist_scans", "hist_z");
        end
        %------------------------------------------------------------------
        function app_st = gen_test_app_st(~, dirty_string, logodds_string, d_t)
            app_st.h1 = figure;
            app_st.axScan = subplot(131);
            app_st.axFeat = subplot(132);
            app_st.axOG = subplot(133);
            app_st.Dirty = dirty_string;
            app_st.LogOdds = logodds_string;
            app_st.dt = d_t;
        end
        %------------------------------------------------------------------
        function export_figures(obj)
            % SCANS AND POSES
            figure;
            ax = gca;
            hold(ax, 'on');
            obj.viz_scans(ax, 2);
            
            % FEAT MAP DIRTY AND POSES
            figure;
            ax1 = gca;
            obj.viz_feat_map(ax1, 'Dirty', 2);
            
            % OG MAP PROB
            figure;
            ax2 = gca;
            obj.viz_og_map(ax2, 'Probabilistic');
            
        end
        %------------------------------------------------------------------
        function recompute_maps(obj)
            % This method should be sufficient given poses and scans exist
            % Results in filling: og_map, scans_transf, feat_map
            % OG MAP
            if all(obj.og_map == 0, 'all') 
                for k=1:size(obj.poses, 2)
                    if ~isempty(obj.hist_z{k})
                        obj.og_map = occupancy_grid(obj.og_map, obj.poses(:, k), obj.hist_z{k}, obj.res, obj.dmax);
                    end
                end
            end
            if all(cellfun(@isempty, obj.scans_transf))
                % ALIGNED SCANS
                obj.scans_transf{1} = applyPoses2Scans(obj.poses, obj.hist_scans);
            end
            
            if iscell(obj.feat_map)
                if all(cellfun(@isempty, obj.feat_map))
                    % FEAT MAP (DIRTY)
                    obj.feat_map = applyPoses2Feats(obj.poses, obj.hist_feat);
                end
            else
                if isempty(obj.feat_map)
                    % FEAT MAP (DIRTY)
                    obj.feat_map = applyPoses2Feats(obj.poses, obj.hist_feat);
                end
            end
                
        end
        %------------------------------------------------------------------
        function process_scans(obj)
            % This method should get raw scans to turn into useful data
            % Results in filling: hist_feat, hist_scans
            obj.hist_scans = {};
            if all(cellfun(@isempty, obj.hist_feat))
                for i=1:size(obj.hist_z, 1)
                    z_aux = obj.hist_z{i};
                    if ~isempty(z_aux)
                        [~, idx] = sort(z_aux(2, :));
                        z_aux = z_aux(:, idx);
                        [xAll, yAll] = pol2cart(z_aux(2, :), z_aux(1, :));
                        data = double([xAll; yAll]');

                        current_data = data(~isinf(data(:, 1)), :);
                        list = my_split_and_merge(current_data, obj.eps);

                        obj.hist_feat{i} = list(:, 1);
                        obj.hist_scans = cat(1, obj.hist_scans, current_data);
                    end
                end
            end
        end
        %------------------------------------------------------------------
        function coverage = coverage_percentage(obj, total_area_2)
            if nargin > 1
                given_area = total_area_2;
            else
                given_area = obj.total_area;
            end
            % Given total area of environment, computes current area known
            covered_cells = nnz(obj.og_map ~= 0);
            covered_area = covered_cells/obj.res^2;
%             disp(covered_area);
            coverage = covered_area/given_area;
        end
        %------------------------------------------------------------------
        function gpose = pos2grid(obj, pose)
            gpose = zeros(size(pose));

            x = pose(1, :);
            y = pose(2, :);
            if(size(pose, 1) > 2)
                gpose(3, :) = pose(3, :);
            end
            gsize = size(obj.og_map);

            RealSize = obj.mapSize/obj.res;

            gpose(1, :) = round(-y.*(gsize(1)/RealSize) + gsize(1)/2);
            gpose(2, :) = round(x.*(gsize(2)/RealSize) + gsize(2)/2);
            gpose = gpose';
        end
        %------------------------------------------------------------------
        function pose = grid2pos(obj, gpose)
            pose = zeros(size(gpose));
            gsize = size(obj.og_map);

            RealSize = obj.mapSize/obj.res;

            pose(:, 1) = (gpose(:, 2) - gsize(2)/2)./(gsize(2)/RealSize);
            pose(:, 2) = -(gpose(:, 1) - gsize(1)/2)./(gsize(1)/RealSize);
        end
        %------------------------------------------------------------------
        function show_path(obj, ax)
            if ~isempty(obj.path)
                imagesc(ax, 1 - flipud(obj.og_map));
                colormap(ax, 'gray');
                hold(ax, 'on');
                plot(ax, obj.path(1, 2), obj.path(1, 1), 'yp', 'MarkerFaceColor', 'y');
                plot(ax, obj.path(end, 2), obj.path(end, 1), 'ro', 'MarkerFaceColor', 'r');
                plot(ax, obj.path(:, 2), obj.path(:, 1), 'k.');
                hold(ax, 'off');
                axis(ax, 'image');
            end
        end
        %------------------------------------------------------------------
        function check_if_stuck(obj)
            % If last and third to last poses are close
            % There are a lot of measurements with same value of distance
            % Then Im stuck
            id = cellfun(@isempty, obj.hist_z);
            z = obj.hist_z(~id);
            z = z{end - 1};
            [N, ~] = histcounts(z(1, :));
            [~, idx] = max(N);
            if (size(obj.poses, 2) > 3) && ~obj.am_i_stuck && (idx == 1)
                obj.am_i_stuck = norm(obj.poses(1:2, end) - obj.poses(1:2, end-2)) < 0.1;
            else
                obj.am_i_stuck = false;
            end
        end
        %------------------------------------------------------------------
        function valid_path = update_path(obj, forbiddenPoint)
            % See test_exploration for debugging
            % Using unoccupied space to generate binary image
            map = flipud(obj.og_map);
            og_map_prob = recover_from_log_odds(map);
            og_map_bin = binaryMap(og_map_prob, 0.4, 0.7);
            
%             og_map_bin = (map == 0);
            GVD_only_skel = bwmorph(og_map_bin == 0, 'skel', Inf);
            GVD_only_thin = bwmorph(og_map_bin == 0, 'thin', Inf);
            GVD_only = GVD_only_skel;
            
            % Remove unconnected pixels in GVD path
            CC = bwconncomp(GVD_only);
            if CC.NumObjects > 1
                aux_GVD_only = false(CC.ImageSize);
                numPixels = cellfun(@numel, CC.PixelIdxList);
                [~, idx] = max(numPixels);
                aux_GVD_only(CC.PixelIdxList{idx}) = true;
                
                GVD_only = aux_GVD_only;
            end
            
            valid_rule1 = 1;
            usedThin = 0;
            while valid_rule1
                skelMap = 1 - GVD_only;
                % Erasing skel cells that are nearby walls
                [r, c] = find(skelMap == 0);
                cells = [r, c];
                for j=1:size(cells, 1)
                    skel_cell = cells(j, :);
                    neighbor_list = get_neighbor(skel_cell, og_map_bin);
                    if nnz(neighbor_list(:, 1) == 1) >= 2
                        skelMap(skel_cell(1), skel_cell(2)) = 1;
                    end
                end

                
                % ========================= Compute Start Node ============================
                % Start will be the knn of current pose on gvd nodes
%                 [i, j] = find(GVD_only == 1);
%                 gvd_cr = [j, i];
%                 gpose = obj.pos2grid(obj.poses(1:2, end));
%                 start_idx = knnsearch(gvd_cr, fliplr(gpose));
%                 pose_gvd = gvd_cr(start_idx, :);
                
                [i, j] = find(GVD_only == 1);
                gvd_cr = [j, i];
                gpose = obj.pos2grid(obj.poses(1:2, end));
                [~, start_idx] = search_cell_in_list(og_map_bin, [i, j], gpose);
                pose_gvd = gvd_cr(start_idx, :);
                if isempty(pose_gvd)
                    % If start is empty, is probably inside a wall, go back
                    valid_path = 0;
                    return;
                end
                
                % ========================= Compute Goal Node =============================
                % Goal will be computed according to method
                % Try this:
                % Rule 1: Get the branch nodes that have 2 end nodes neighbors
                % Rule 2: The cost (utility) will be the distance from current pose to each
                % candidate minus the distance to nearest frontier cell
                % Push to PQ
                % After all branch nodes from rule 1 are inserted, the goal will be the
                % pq.pop result (the one with the least utility)

                % RULE 1 <========================
                % Generate graph from skel
                % The objective is to filter nodes through rule 1 using a graph structure
                [rule1, targets_to_frontier] = compute_rule1(skelMap);
                
                %{
                [~, node, ~] = Skel2Graph3D(1 - skelMap, 0);
                % #########################################################
                % OBS: maybe use the edge length give in here for later
                % filter rule1->frontier distances greater than edges to
                % end points
                % #########################################################
                rule1 = [];
                for j=1:length(node)
                    nd = node(j);
                    if length(nd.conn) > 2
                        if nnz([node(nd.conn).ep]) >= 2
                            [r, c] = ind2sub(size(skelMap), nd.idx);
                            rule1 = cat(1, rule1, [r, c]);
                        end
                    end
                end
                
                %}
                if ~isempty(rule1)
                    %{
                    % Cluster rule1 points to remove close points
                    T = clusterdata(rule1, 1);
                    new_rule1 = zeros(max(T), 2);
                    for t=1:max(T)
                        group_nodes = rule1(T == t, :);
                        new_rule1(t, :) = group_nodes(1, :);
                    end
                    rule1 = new_rule1;
                    %}
                    break;
                else
                    if usedThin
                        disp('Trying to change skel 3 times!');
                    end
                    GVD_only = GVD_only_thin;
                    usedThin = 1;
                end
            end
            
            % RULE 2 <========================
            % FIX THIS: frontier cells are no longer needed, find a way to
            % use it
            frontier_list = compute_frontier_cells(og_map_bin);
            
            if ~isempty(frontier_list)
                % Get knn frontier cells from targets candidates as well as their distances
%                 [goal_idx, targets_to_frontier] = knnsearch(fliplr(frontier_list), fliplr(rule1), 'Distance', 'cityblock');
%                 frontier_knn = frontier_list(goal_idx, :);

                % Get distance from start to each rule1 candidate
                start = fliplr(pose_gvd);
                start_to_targets = zeros(size(rule1, 1), 1);
                pq = zeros(size(rule1, 1), 1);
                hist_paths = cell(size(rule1, 1), 1);
                
                poses_old = obj.pos2grid(obj.poses(1:2, :));
                wp_radius_grid = obj.wp_radius * obj.res * 1; % Maybe TUNE THIS <<<<<
                for i=1:size(rule1, 1)
                    % Check if target was already visited, either:
                    % By older targets chosen or
                    % By older poses
                    rule1_visited = 0;
                    
                    rule1_to_targets_old = vecnorm(rule1(i, :) - obj.targets, 2, 2);
                    rule1_to_poses_old = vecnorm(rule1(i, :) - poses_old, 2, 2);

                    if any(rule1_to_targets_old <= wp_radius_grid) || any(rule1_to_poses_old <= wp_radius_grid)
                        rule1_visited = 1;
                        obj.targets = cat(1, obj.targets, rule1(i, :));
                    end
                    
                    if size(obj.targets, 1) > 1
                        for t=1:size(obj.targets, 1)
                            target_to_old = norm(rule1(i, :) - obj.targets(t, :));
                            % ATTENTION: Visited targets radius' doubled!
                            if (target_to_old <= (obj.wp_radius*obj.res*2))
                                rule1_visited = 1;
                            end
                        end
                    end

                    if rule1_visited
                        pq(i) = Inf;
                        continue;
                    end
                    % Compute path and distance to rule1 candidate
%                     [new_path, ~, costMap] = a_star(skelMap, start, rule1(i, :), 2);
                    [new_path, ~, costMap] = greedy_search(skelMap, start, rule1(i, :), 1);
                    if isempty(new_path)
                        pq(i) = Inf;
                        continue;
                    end
                    
                    % FOR A BFS SEARCH (C = initial pose->rule1 )
%                     center = obj.pos2grid([0; 0]);
%                     C = norm(center - fliplr(rule1(i, :)));
                    
                    % FOR A DFS SEARCH (C = [start->rule1] + [rule1->frontier])
%                     start_to_targets(i) = costMap(rule1(i, 1), rule1(i, 2), 2);
%                     C = start_to_targets(i) - targets_to_frontier(i);
                    
                    % More elegant way. Inspired by: Voronoi-Based 
                    % Multi-Robot Autonomous  Exploration in Unknown 
                    % Environments via Deep  Reinforcement Learning
%                     start_to_targets(i) = costMap(rule1(i, 1), rule1(i, 2), 2);
%                     start_to_targets(i) = costMap(start(1), start(2), 2);
                    start_to_targets(i) = size(new_path, 1);
                    
                    % 0.7 bugs 2
                    % 0.6 bugs 3
                    % 0.8 goes well until it shifts controllers
                    % 0.9
                    % 1 goes well for small map
                    lambda = 1;
%                     d = start_to_targets(i) + targets_to_frontier(i);
                    d = start_to_targets(i);
                    phi = norm(obj.pos2grid([0; 0]) - fliplr(rule1(i, :)));
                    
%                     d = targets_to_frontier(i);
%                     phi = start_to_targets(i);
                    
                    C = lambda * d + (1 - lambda) * phi;
                    
                    % Conditions to discard a target
                    if (C <= 0)
                        C = Inf;
                    end
                    % C is in grid units
                    pq(i) = C;
                    hist_paths{i} = new_path;
                end
                
                % Get goal from PQ and A* path
                [~, chosen_rule1] = min(pq);
                
                % Check if already inside goal waypoint radius
                start_to_goal = norm((obj.grid2pos(rule1(chosen_rule1, :)))' - obj.poses(1:2, end));
                if start_to_goal <= obj.wp_radius
                    pq(chosen_rule1) = Inf;
                    [~, chosen_rule1] = min(pq);
                end
                
                % Check if new goal is too close to old path
                if ~isempty(obj.cont.Waypoints)
                    [~, old_path_to_goal] = knnsearch(obj.cont.Waypoints, obj.grid2pos(rule1(chosen_rule1, :)));
                    if nnz(old_path_to_goal <= obj.wp_radius)
                        pq(chosen_rule1) = Inf;
                        [~, chosen_rule1] = min(pq);
                    end
                end
                
                % If all paths returned empty
                if all(cellfun(@isempty, hist_paths))
                    % Compute path to random frontier or rule1?
                    if size(rule1, 1) == 1
                        goal = rule1;
                    else
                        rand_sample = datasample(rule1, 1);
                        goal = rand_sample;
                    end
%                     [new_path, ~, ~] = greedy_search(og_map_bin ~= 0, start, goal, 1);
                    [new_path, ~, ~] = a_star_cost_layer(og_map_bin ~= 0, skelMap == 0, start, goal, 2);
                else
                    if nargin > 1
                        while norm(obj.grid2pos(rule1(chosen_rule1, :)) - forbiddenPoint) < 2
                            % Get another one
%                             rule1(chosen_rule1, :) = [];
                            pq(chosen_rule1) = Inf;
                            if isempty(rule1)
                                disp('thats a problem right there...');
                            else
                                [~, chosen_rule1] = min(pq);
                            end
                            
                        end
                    end
                    if all(isinf(pq))
                        chosen_rule1 = find(~cellfun(@isempty,hist_paths));
                    end
                    % Get chosen path from previously computed paths
                    new_path = hist_paths{chosen_rule1};
                    goal = rule1(chosen_rule1, :);
                    
                end

                % Finally, update exploration variables variables
                obj.path = new_path;
                obj.cont.Waypoints = obj.grid2pos(flipud(new_path));
                obj.targets = cat(1, obj.targets, goal);
                valid_path = 1;
                
                % DEBUG STARTS HERE ---------------------------------------
                n = obj.n_r;
                h1 = figure(1);
%                 xlim_xy = h1.Children((2*n+1)-obj.exp).XLim;
%                 ylim_xy = h1.Children((2*n+1)-obj.exp).YLim;
%                 xlim_gr = obj.pos2grid([xlim_xy; 0, 0]);
%                 ylim_gr = obj.pos2grid([0, 0; ylim_xy]);
%                 xlim_gr = [xlim_gr(1, 2), xlim_gr(2, 2)];
%                 ylim_gr = [ylim_gr(2, 1), ylim_gr(1, 1)];
                
                subplot(2, n, obj.exp+n);
                imshow(1 - og_map_bin);
                hold on;
                plot(gpose(2), gpose(1), 'rx', 'MarkerSize', 7, 'DisplayName', 'Pose');
                plot(gvd_cr(:, 1), gvd_cr(:, 2), 'm.', 'DisplayName', 'GVD path');
                plot(pose_gvd(1), pose_gvd(2), 'r+', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'A* start');
                plot(frontier_list(:, 2), frontier_list(:, 1), 'b.', 'DisplayName', 'Frontier cells');
%                 plot(frontier_knn(:, 2), frontier_knn(:, 1), 'bo', 'MarkerSize', 8, 'DisplayName', 'Frontier KNN');
                plot(obj.path(:, 2), obj.path(:, 1), 'r.');
                for rl=1:size(rule1, 1)
                    plot(rule1(rl, 2), rule1(rl, 1), 'g+', 'LineWidth', 2, 'DisplayName', 'Rule 1 cells');
                    text(rule1(rl, 2), rule1(rl, 1), ['\leftarrow ', num2str(pq(rl))], 'FontWeight', 'bold');
                end
                plot(goal(2), goal(1), 'rp', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', 'A* goal');
%                 for i=1:size(rule1, 1)
%                     plot([rule1(i, 2); frontier_knn(i, 2)], [rule1(i, 1); frontier_knn(i, 1)], 'c-', 'HandleVisibility', 'off');
%                 end
                if nargin > 1
                    avoidPt = obj.pos2grid(forbiddenPoint');
                    plot(avoidPt(2), avoidPt(1), 'bh', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'DisplayName', 'Avoid this');
                end
                hold off;
                axis equal;
                grid on;
%                 legend('Location', 'Best');
                if usedThin
                    title('Using thin');
                else
                    title('Using skel');
                end
%                 xlim(xlim_gr + [-50, 50]);
%                 ylim(ylim_gr + [-50, 50]);
                
            else
                % Frontier list empty
                valid_path = 0;
                
                disp('Frontier list EMPTY! Finish exploration task');
                
            end
        end
        %------------------------------------------------------------------
        function move_and_stop(obj, dt)
            % move
            obj.ROS_params.twist_msg.Linear.X = obj.u(1, end);
            obj.ROS_params.twist_msg.Angular.Z = obj.u(2, end);       
            send(obj.ROS_params.cmd, obj.ROS_params.twist_msg);
            % delay
            pause(dt);
            % stop
            obj.ROS_params.twist_msg.Linear.X = 0;
            obj.ROS_params.twist_msg.Angular.Z = 0;
            send(obj.ROS_params.cmd, obj.ROS_params.twist_msg);
            if obj.mode == 4
                pause(1);
            end
        end
        %------------------------------------------------------------------
        function w = dir2w(obj, steeringDir)
            % Given steering direction in rad, compute angular rate w in
            % rad/s
            wMax = obj.maxW;
            % Computing in robot's coordinate frame
            curPose = [0 0 0];

            % The following computation is similar to PurePursuit
            lookaheadPoint = [cos(steeringDir), sin(steeringDir)];
            slope = atan2((lookaheadPoint(2) - curPose(2)), (lookaheadPoint(1) - curPose(1)));
            alpha = angdiff(curPose(3), slope);

            % Angular velocity command for a differential drive robot is
            % equal to the desired curvature to be followed by the robot
            w = (2*sin(alpha));

            % Pick a constant rotation when robot is facing in the opposite
            % direction of the path
            if abs(abs(alpha) - pi) < 1e-12
                w = sign(w)*1;
            end

            if abs(w) > wMax
                w = sign(w)*wMax;
            end
        end
        %------------------------------------------------------------------
        function [switched] = check_conflict_zone(obj, r2, T)
            % Given another TaskStructure object r2 and transformation T,
            % check if this object is in conflict by distance thresholding
            pose1 = obj.poses(1:2, end);
            pose2 = r2.poses(1:2, end);

            pose2_t = T * [pose2; 1];
            switched = 0;
            if norm(pose1 - pose2_t(1:2)) < obj.r_collision
                if strcmp(obj.active_cont, 'Global')
                    switched = 1;
                end
                obj.active_cont = 'Local';
                r2.active_cont = 'Local';
            else
                if strcmp(obj.active_cont, 'Local')
                    switched = 2;
                end
                obj.active_cont = 'Global';
                r2.active_cont = 'Global';
            end
            
        end
         %------------------------------------------------------------------
        function [iSeeYou, rel, ID] = detect_robot_tag(obj)
            if strcmp(obj.name, 'Isis')
%                 cameraInfoSubs = obj.ROS_params.camera_info;
%                 rgbImageSubs = obj.ROS_params.camera_rgb;
%                 tagSize = 0.105;
%                 offset = [0, 0, 0, 0];
%                 [iSeeYou, rel, ID] = detect_tag_camera_ros(cameraInfoSubs, rgbImageSubs, tagSize, offset);
                
                msg = receive(obj.ROS_params.rel_info);
                rel_info = msg.Data;
                
                if isempty(rel_info)
                    iSeeYou = 0;
                    rel = [];
                    ID = [];
                else
%                     disp(['Tag detected at ', mat2str(rel_info)]);
                    iSeeYou = 1;
                    rel = rel_info;
                    ID = [];
                    obj.relative_info = rel;
                end
            elseif strcmp(obj.name, 'Omni')
                
                msg = receive(obj.ROS_params.rel_info);
                rel_info = msg.Data;
                
                if isempty(rel_info)
                    iSeeYou = 0;
                    rel = [];
                    ID = [];
                else
%                     disp(['Tag detected at ', mat2str(rel_info)]);
                    iSeeYou = 1;
                    rel = rel_info;
                    ID = [];
                    obj.relative_info = rel;
                end
            else
                iSeeYou = 0;
                rel = [];
                ID = [];
            end
        end
    end
end