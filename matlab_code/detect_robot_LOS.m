% Detecting nearby robots in LOS
% This function should give necessary information for merging on the spot
% Transformation is from r1 to r2
% input: ts = cell array of TaskStructure objects
%        isSim = boolean to check if using simulation routines
% output: proximityOrder = {n, 3} cell array, with {r1 index, r2 index, homogeneous transformation T} as columns
function [proximityOrder] = detect_robot_LOS(ts, isSim)
    proximityOrder = {};
    n = length(ts);
    dmax = (ts{1}.dmax)*.7;  % Still need to tune this
    
    if isSim
        % Compute GT poses of all robots to check relative positions
        points = zeros(3, n);
        for i=1:n
            points(:, i) = ts{i}.gt_poses(:, end);
        end
        % Check if any points are in range
        D = pdist(points(1:2, :)');
        Z = squareform(D);
        [r1, r2] = find((Z < dmax) & (Z ~= 0));
        if ~isempty(r1)
            % This will check the inverse order, since Z is symmetrical
            for j=1:size(r1, 1)
                gt_pose1 = ts{r1(j)}.gt_poses(:, end);
                gt_pose2 = ts{r2(j)}.gt_poses(:, end);
                pose1 = ts{r1(j)}.poses(:, end);
                pose2 = ts{r2(j)}.poses(:, end);
                
                % Extract transformation from GT poses
                [Rr, tr, ~] = poses2transf(gt_pose1, gt_pose2);
                % Robot1 inverse transformation using (R*p + t) form
%                 R1_inv = eul2rotm([-pose1(3), 0, 0]); R1_inv = R1_inv(1:2, 1:2);
%                 t1_inv = -pose1(1:2);
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
                
                % Apply transform to grid and cartesian pose1
%                 pose1_grid = ts{r1(1)}.pos2grid(pose1);
                pose2_grid = ts{r2(2)}.pos2grid(pose2(1:2));
                pose1_transf = T * [pose1(1:2); 1];
                pose1_transf_grid = ts{r1(1)}.pos2grid(pose1_transf(1:2));
                
                % Compute LOS cells using bresenham algorithm
                los = bresenham(pose1_transf_grid', pose2_grid');
                % Check robot2 if all cells within LOS are free
                indLOS = sub2ind(size(ts{r2(j)}.og_map), los(1, :), los(2, :));
                map_flipped = flipud(ts{r2(j)}.og_map);
                robotDetected = all(map_flipped(indLOS) < 0);
                if robotDetected
                    proximityOrder = cat(1, proximityOrder, {r1(j), r2(j), T});
%                     
%                     figure;
%                     ts{r2(j)}.viz_og_map(gca, 'Probabilistic');
%                     hold(gca, 'on');
%                     plot(pose1_transf_grid(2), pose1_transf_grid(1), 'r+', 'LineWidth', 2);
%                     plot(pose2_grid(2), pose2_grid(1), 'b+', 'LineWidth', 2);
%                     plot([pose1_transf_grid(2), pose2_grid(2)], [pose1_transf_grid(1), pose2_grid(1)], 'm--');
%                     plot(los(2, :), los(1, :), 'gs');
%                     title(gca, {'SLAM relations', '\color{green}Green\color{black} is LOS on grid', '\color{magenta}Magenta\color{black} is LOS on cartesian'});
                end
                
            end
        else
            % No robot within reach
            return;
        end
    else
        % Visual detection routine using robots camera
        % TODO: implement part where they face each other and overcome
        % distance threshold
        
        [iSeeYou_omni, rel_omni, ~] = ts{1}.detect_robot_tag();
        [iSeeYou_isis, rel_isis, ~] = ts{2}.detect_robot_tag();
        if iSeeYou_isis && ~iSeeYou_omni
            disp(['Isis saw Omni ', mat2str(rel_isis)]);
            % Put Omni on direct path to Isis
        elseif iSeeYou_omni && ~iSeeYou_isis
            disp(['Omni saw Isis ', mat2str(rel_omni)]);
            % Put Isis on direct path to Omni
%             ts{2}.path = pos2grid
        end
        
        if iSeeYou_isis && iSeeYou_omni && all([rel_isis(1), rel_omni(1)] < 2.5) % tune this <============
        
            pose1 = ts{1}.poses(:, end);
            pose2 = ts{2}.poses(:, end);

            % Try to get more information for r as possible, then apply
            % some sort of averaging and/or outlier removal to improve
            % measurement. r should be an array of measurements to be
            % averaged r = [from tag1, from tag2, from RGBD1, from RGBD2, ...]
            r = mean([rel_isis(1), rel_omni(1)]);
            theta2_rel = rel_isis(2);
            theta1_rel = rel_omni(2);

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
        end
        

    end
end

% Function description: Bresenham line algorithm (https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm)
%===============================================================================
% INPUT:
% @start_pnt    start point of the line
% @end_pnt      end point of the line
% OUTPUT:
% @l            points on line 
% DATE:         2018/12/23 wyq
%===============================================================================

function l = bresenham(start_pnt, end_pnt)
    
    x0 = start_pnt(1);
    x1 = end_pnt(1);
    y0 = start_pnt(2);
    y1 = end_pnt(2);
    
    dx = x1-x0;
    dy = y1-y0;
    
    if abs(dx) >= abs(dy)
        da = abs(dx);
        db = abs(dy);
        err = abs(db)/2;
        dira = sign(dx);
        dirb = sign(dy);        
    else
        da = abs(dy);
        db = abs(dx);
        err = abs(db)/2;
        dira = sign(dx);
        dirb = sign(dy);
    end
    
    l = zeros(2,3000);
    j = 0;
    cnt = 1;
    for i = 0:da-1
        err = err + db;
        if err >= da
            j = j + 1;
            err = err - da;
        end 
       
        if abs(dx) >= abs(dy)   
            l(:, cnt) = start_pnt + [dira*i; dirb*j];
            cnt = cnt + 1;
        else
            l(:, cnt) = start_pnt + [dira*j; dirb*i];
            cnt = cnt + 1;
        end
    end  
    l(:, cnt:end) = [];    
end

