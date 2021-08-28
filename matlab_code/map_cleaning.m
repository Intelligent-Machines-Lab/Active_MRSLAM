% Map cleaning on features
% Applying the relations between segments, fuse into one common feature
% input: all_feat = features already aligned as a batch of local maps
%        isBuonocore = if 1, uses Buonocore way of fusing, if 0, uses mine
% output: global_map = single feature map containing all fused local maps
function global_map = map_cleaning(all_feat, isBuonocore, params)
    r1_tol = params.r1_tol;
    r4_tol = params.r4_tol;
    
    
    % LINE SEGMENT MATCHING--------------------------------------------
    % Trying to fuse lines using geometric relations
    % Pairing each segment on scan1 to all on scan2, if passes some rules
    % of matching, the segments are fused
    debug = 0;
    fused_segments = {};
    for i=1:size(all_feat)-1
        if isempty(fused_segments)
            scan1 = all_feat{i};
        else
            scan1 = fused_segments;
            fused_segments = {};
        end
        scan2 = all_feat{i+1};
        
        used_idx = [];
        for m=1:size(scan1, 1)
            segment1 = scan1{m};
            for s=1:size(scan2, 1)
                segment2 = scan2{s};
                if isBuonocore
                    % Buonocore's way
                    % It has some relations that mine doesnt
                    fused_line = fuse_line_buonocore(segment1, segment2, 0, params);
                    if ~isempty(fused_line)
                        used_idx = cat(1, used_idx, [m, s]);
                        fused_segments = cat(1, fused_segments, fused_line);
                    else
                        continue;
                    end
                else
                    % My way
                    % Extract relations
                    [r1, ~, ~, r4] = relation(segment1, segment2);
                    r4 = abs(r4 - 4);
                    r1 = abs(r1);
                    % If lines are almost collinear and same size
                    if (r1 < r1_tol) && (r4 < r4_tol)
                        data = [segment1; segment2];

                        [fused_seg, ok] = LS_line(data, 0);
                        if(~ok)
                            continue;
                        end
                        used_idx = cat(1, used_idx, [m, s]);
                        fused_segments = cat(1, fused_segments, fused_seg);
                    end
                end
            end
        end
        % If there was a fusion
        if ~isempty(fused_segments)
            % Verify repeated values to be fused again
            [U1, ia1, ~] = unique(used_idx(:, 1), 'stable');
            [U2, ia2, ~] = unique(used_idx(:, 2), 'stable');
            repeated1 = length(U1) ~= length(used_idx(:, 1));
            repeated2 = length(U2) ~= length(used_idx(:, 2));
            
            % repeated1 and 2 are flags to analyse scan1 or 2 for repetition
            if repeated1
                rep = used_idx(:, 1);
                rep(ia1) = [];
                n = length(rep);
                idx = cell(n, 1);
                idx1 = cell(n, 1);
                old = 0;
                % rep will contain all indexes values that repeat
                for j=1:n
                    if old == rep(j)
                        continue;
                    end
                    % idx will contain logic index array to fuse
                    idx{j} = (used_idx(:, 1) == rep(j));
                    [fused_seg, ok] = LS_line([cell2mat(scan1(used_idx(idx{j}, 1))); cell2mat(scan2(used_idx(idx{j}, 2)))], 0);
                    if ok
                        % idx1 will later be used to delete the fusion
                        idx1{j} = cat(1, idx{j}, false(j, 1));
                        fused_segments = cat(1, fused_segments, fused_seg);
                    end
                    old = rep(j);
                end
            end
            % Same thing
            if repeated2
                rep = used_idx(:, 2);
                rep(ia2) = [];
                n = length(rep);
                idx = cell(n, 1);
                idx2 = cell(n, 1);
                old = 0;
                for j=1:n
                    if old == rep(j)
                        continue;
                    end
                    idx{j} = (used_idx(:, 2) == rep(j));
                    [fused_seg, ok] = LS_line([cell2mat(scan1(used_idx(idx{j}, 1))); cell2mat(scan2(used_idx(idx{j}, 2)))], 0);
                    if ok
                        idx2{j} = cat(1, idx{j}, false(j, 1));
                        fused_segments = cat(1, fused_segments, fused_seg);
                    end
                    old = rep(j);
                end
            end
            % Deleting where there was fusion
            if repeated1 && repeated2
                n1 = length(idx1{1});
                n2 = length(idx2{1});
                aux1 = idx1{1};
                aux2 = idx2{1};
                for j=1:length(idx1)
                    if isempty(idx1{j})
                        continue;
                    end
                    aux1 = aux1 | idx1{j}(1:n1);
                end
                for j=1:length(idx2)
                    if isempty(idx2{j})
                        continue;
                    end
                    aux2 = aux2 | idx2{j}(1:n2);
                end
                idx1End = false(size(fused_segments, 1), 1); idx1End(1:n1) = aux1;
                idx2End = false(size(fused_segments, 1), 1); idx2End(1:n2) = aux2;
                idx = (idx1End | idx2End);
                fused_segments(idx) = [];
            elseif repeated1 && ~repeated2
                n1 = length(idx1{1});
                aux1 = idx1{1};
                for j=1:length(idx1)
                    if isempty(idx1{j})
                        continue;
                    end
                    aux1 = aux1 | idx1{j}(1:n1);
                end
                idx1End = false(size(fused_segments, 1), 1); idx1End(1:n1) = aux1;
                fused_segments(idx1End) = [];
            elseif ~repeated1 && repeated2
                n2 = length(idx2{1});
                aux2 = idx2{1};
                for j=1:length(idx2)
                    if isempty(idx2{j})
                        continue;
                    end
                    aux2 = aux2 | idx2{j}(1:n2);
                end
                idx2End = false(size(fused_segments, 1), 1); idx2End(1:n2) = aux2;
                fused_segments(idx2End) = [];
            end
            
            % Tried not to modify the used_idx because its used below, but
            % i think it might be necessary in the future.
            % Add elements not paired
            mm = ~ismember(1:size(scan1, 1), used_idx(:, 1));
            nn = ~ismember(1:size(scan2, 1), used_idx(:, 2));
            if any(mm)
                fused_segments = cat(1, fused_segments, scan1(mm));
            end
            if any(nn)
                fused_segments = cat(1, fused_segments, scan2(nn));
            end
        else
            % If there wasnt a fusion, concatenate scan 1 and 2
            fused_segments = cat(1, scan1, scan2);
        end
        
        % VIZ==============================================================
        if debug
            figure;
            subplot(131);
            hold on;
            for m=1:size(scan1, 1)
                segment = scan1{m};
                plot(segment(:, 1), segment(:, 2), 'rs-');
            end
            hold off;
            grid on;
            axis equal;
            title('Scan 1');
            subplot(132);
            hold on;
            for m=1:size(scan2, 1)
                segment = scan2{m};
                plot(segment(:, 1), segment(:, 2), 'ro-');
            end
            hold off;
            grid on;
            axis equal;
            title('Scan 2');
            subplot(133);
            hold on;
            for m=1:size(fused_segments, 1)
                segment = fused_segments{m};
                plot(segment(:, 1), segment(:, 2), 'ko-');
            end
            hold off;
            grid on;
            axis equal;
            title('Fused features using relations');
        end
    end
    global_map = fused_segments;
end


% Output segment fit with LS
% ======================================
% This is probably the definitive version for fitting lines using LS
% ======================================
function [seg, ok] = LS_line(data, debug)
    seg = [];
    ok = 1;
    
    [a, b, c] = fit_line_hesse(data);
    
    if any(isempty([a, b, c]))
        disp('Could not fit line!');
        ok = 0;
    else
        if debug
            figure;
            hold on;
            plot(data(1:2, 1), data(1:2, 2), 'rx-');
            plot(data(3:4, 1), data(3:4, 2), 'mx-');
        end
        % Compose line segment using coefficients
        % Get max and min on y
        y1 = max(data(:, 2));
        y2 = min(data(:, 2));
        x1 = (c - b * y1) / a;
        x2 = (c - b * y2) / a;
        if debug
            plot([x1, x2], [y1, y2], 'gx-');
        end
        % If line is not consistent with the max length of segments
        if(abs(x1 - x2) > max(pdist(data)))
            x1 = max(data(:, 1));
            x2 = min(data(:, 1));
            y1 = (c - a * x1) / b;
            y2 = (c - a * x2) / b;
            if debug
                plot([x1, x2], [y1, y2], 'bx-');
            end
        end
        if debug
            hold off;
            grid on;
            axis equal;
        end
        
        seg = double([x1, y1; x2, y2]);
    end
end

% Extract relations between 2 segments as seen in:
% A Line Feature Matching Technique Based on an Eigenvector Approach
% input: x1 = [x, y; x, y], x2 = [x, y; x, y]
% output: 4 relations, see paper
function [r1, r2, r3, r4, st] = relation(x1, x2)
    v1 = x1(1, :) - x1(2, :);
    v2 = x2(1, :) - x2(2, :);
    c1 = mean(x1);
    c2 = mean(x2);
    MN = c2 - c1;
    
    r1 = ang_vec(v1, v2);
    r2 = ang_vec(MN, v1);
    % Trying to fix the close-to-pi problem by getting the smaller angle
    r1 = min(r1, pi-r1);
    r2 = min(r2, pi-r2);
    
    AB = norm(v1);
    CD = norm(v2);
    AC = norm(x1(1, :) - x2(1, :));
    AD = norm(x1(1, :) - x2(2, :));
    BC = norm(x1(2, :) - x2(1, :));
    BD = norm(x1(2, :) - x2(2, :));
    d = (AC + AD + BC + BD)/4;
    
    r3 = AB/CD;
    r4 = (AB + CD)/d;
    if nargout > 4
        st.AB = AB;
        st.CD = CD;
        st.AC = AC;
        st.AD = AD;
        st.BC = BC;
        st.BD = BD;
    end
end

% Angle between 2 vectors in rad
function ang = ang_vec(v1, v2)
    v1 = [v1, 0];
    v2 = [v2, 0];
%     ang = atan2(norm(cross(v1,v2)), dot(v1,v2));
    ang = atan(norm(cross(v1,v2))/dot(v1,v2));
end

% Fuse lines algorithm using relations inspired by Buonocore's work
function [fused_line] = fuse_line_buonocore(segment1, segment2, debug, params)
    
    d_min = params.d_min;
    d_ort = params.d_ort;
    r1_tol = params.r1_tol_2;

    fused_line = []; 
    
    [r1, ~, ~, ~, st] = relation(segment1, segment2);
    
    AC = st.AC;
    AD = st.AD;
    BC = st.BC;
    BD = st.BD;
    
    
    r1 = abs(r1);
    % If lines are almost collinear
    if (r1 < r1_tol)
        pt = segment1(1, :);
        v1 = segment2(1, :);
        v2 = segment2(2, :);
        d = point_to_line(pt, v1, v2);
        pt_int = line_intersect(segment1, segment2);
%         PA = norm(pt_int - segment1(1, :));
%         PB = norm(pt_int - segment1(2, :));
%         PC = norm(pt_int - segment2(1, :));
%         PD = norm(pt_int - segment2(2, :));
%         pt_int_dist = [PA, PB, PC, PD];
        ok1 = check_points_inline(pt_int, segment1, 0, params);
        ok2 = check_points_inline(pt_int, segment2, 0, params);
        % If one of the vertices are within the orthogonal distance rule
        % Or the distances between vertices are within the min dist rule
        % Or the intersection is within the segments
        if (d < d_ort) || (min([AC, AD, BC, BD]) < d_min) || (ok1 || ok2)
            [seg, ok] = LS_line([segment1; segment2], debug);
            if ~ok
                disp('See here: fuse_line_buonocore');
                fused_line = [];
            else
                fused_line = seg;
            end
        end
    end
end

% Creates a square formed by the segment points and check for points
% inside, if there are, creates a distance threshold over the segment to
% check for points inside
function [ok, idxInliers] = check_points_inline(pts, segment, debug, params)
    xmin = min(segment(:, 1));
    xmax = max(segment(:, 1));
    ymin = min(segment(:, 2));
    ymax = max(segment(:, 2));
    
    % Some tolerance on limits for the square
%     little = 0.01;
%     isVertical = 0.02;
%     isHorizontal = 0.02;
    little = params.little;
    isVertical = params.isVertical;
    isHorizontal = params.isHorizontal;
    
    if abs(xmin-xmax) < isVertical
        % If vertical segment
        xok = (pts(:, 1) < (xmax+little)) & (pts(:, 1) > (xmin-little));
        yok = (pts(:, 2) < ymax) & (pts(:, 2) > ymin);
    elseif abs(ymin-ymax) < isHorizontal
        % If horizontal segment
        xok = (pts(:, 1) < xmax) & (pts(:, 1) > xmin);
        yok = (pts(:, 2) < (ymax+little)) & (pts(:, 2) > (ymin-little));
    else
        % If diagonal segment
        xok = (pts(:, 1) < xmax) & (pts(:, 1) > xmin);
        yok = (pts(:, 2) < ymax) & (pts(:, 2) > ymin);
    end
    
    % idx will be the points inside the square formed by segment
    idx = (xok & yok);
    
    if nargout > 1
        idxInliers = idx;
    end
    
    % if idx is not empty, verify the threshold for inline
    n = find(idx);
    c = 0;
    j = [];
    % distance threshold over the segment
%     dth = 0.05;
    dth = params.dth;
    % maximum allowed for number of inliers
%     cmax = 0;
    cmax = params.cmax;
    if ~isempty(n)
        for i=n'
            d = point_to_line(pts(i, :), segment(1, :), segment(2, :));
            if d < dth
                j = cat(1, j, i);
                c = c + 1;
            end
        end
        % it will be ok if theres more than cmax inliers
%         disp(c)
        ok = (c > cmax);
    else
%         disp('Not verified');
        ok = 0;
    end
    
    if debug
        figure;
        hold on;
        plot(pts(:, 1), pts(:, 2), 'r.');
        if ~isempty(j)
            plot(pts(j, 1), pts(j, 2), 'bo');
        end
        plot(segment(:, 1), segment(:, 2), 'mx-');
        hold off;
        axis equal;
        grid on;
    end
    
end



