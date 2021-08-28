% Read intel dataset
% Got it here:
% https://www.ipb.uni-bonn.de/datasets/

function scans = read_dataset_intel(whichOne)
    if whichOne == 1
        intel_log = readmatrix('intel.log', 'FileType', 'text', 'HeaderLines', 9, 'Delimiter', ' ');
        angles = linspace(-pi/2, pi/2, 180);
        scans = {};
        for i=1:size(intel_log, 1)
            l = intel_log(i, :);
            if l(2) == 180
                ranges = l(3:182);
    %             ranges(ranges == 80) = Inf;
                scans = cat(1, scans, [ranges; angles]);
            end
        end
    elseif whichOne == 2
        intel_log = readcell('intel.gfs.log', 'FileType', 'text', 'Delimiter', ' ');
        angles = linspace(-pi/2, pi/2, 180);
        scans = {};
        for i=1:size(intel_log, 1)
            l = intel_log(i, :);
            if strcmp(l{1}, 'FLASER')
                ranges = cell2mat(l(3:182));
    %             ranges(ranges == 80) = Inf;
                scans = cat(1, scans, [ranges; angles]);
            end
        end
    elseif whichOne == 3
        intel_log = readcell('intel.gfs', 'FileType', 'text', 'Delimiter', ' ');
        angles = linspace(-pi/2, pi/2, 180);
        scans = {};
        for i=1:size(intel_log, 1)
            l = intel_log(i, :);
            if strcmp(l{1}, 'LASER_READING')
                ranges = cell2mat(l(3:182));
    %             ranges(ranges == 80) = Inf;
                scans = cat(1, scans, [ranges; angles]);
            end
        end
    elseif whichOne == 4
        intel_log = readcell('input_INTEL_g2o.g2o', 'FileType', 'text', 'Delimiter', ' ');
        pg = myPoseGraph([]);
        for i=1:size(intel_log, 1)
            l = intel_log(i, :);
            if strcmp(l{1}, 'VERTEX_SE2')
                pose = [l{3:5}]';
                id = l{2};
                pg.add_node_new(pose, id);
            elseif strcmp(l{1}, 'EDGE_SE2')
                idFrom = l{2}; idTo = l{3};
                z = [l{4:6}]';
                I1_3 = [l{7:9}]; I2_3 = [l{10:11}]; I33 = l{12};
                I = [I1_3; I1_3(2), I2_3; I1_3(3), I2_3(2), I33];
                
                pg.add_edge_new(z, idFrom, idTo, I);
            end
        end
        scans = pg;
    end
end

% figure;
% for j=1:size(scans, 1)
%     test = scans{j};
%     [x, y] = pol2cart(test(2, :), test(1, :));
%     
%     plot(x, y, 'r.');
%     axis equal;
%     grid on;
%     pause(0.1)
%     cla(gca);
% end

