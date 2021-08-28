% Trigger all the necessary stuff to return a measurement array
% input: flag = boolean ROS msg
%        pub_flag = topic /do_scan already defined as publisher
%        depth_scan = topic /depth_scan already defined as subscribe
% output: z = [d, phi]' array of measurements
function z = start_scan_omni(flag, pub_flag, depth_scan)
    % User defined constants
    %              1    2    3    4    5    6    7
    ALL_WIDTH = [1280, 848, 640, 640, 480, 424, 212];
    ALL_HEIGHT = [720, 480, 480, 360, 270, 240, 180];
    
    i = 7;
    
    depth_width = ALL_WIDTH(i);
    depth_height = ALL_HEIGHT(i);
    % if depth_width == ALL_WIDTH(2)
    %     quadrants = 4;
    %     timeOut = 20; % find lower value
    % elseif depth_width == ALL_WIDTH(7)
    %     quadrants = 8;
    %     timeOut = 50; % find lower value
    % end

    quadrants = 4;
    HFOV = 90;
    timeOut = 25; % find lower value
%     spacing = (2 * pi) / quadrants;
    

    % Each measurement follows this angle specification
%     inc = HFOV/(depth_width - 1);
%     angles_deg = HFOV/2 : -inc : -HFOV/2;

    % Angles for 4 measurements, if more is needed create a (n, depth_width) array
%     angles_rad = zeros(quadrants, depth_width);
%     angles_rad(1, :) = deg2rad(angles_deg);
%     angles = angles_rad(1, :);
%     for i=1:quadrants-1
%         angles_rad(i+1, :) = angles_rad(i, :) + spacing;
%         angles = cat(2, angles, angles_rad(i+1, :));
%     end
%     % figure; polarplot(angles, ones(1, length(angles)), 'r.');
%     new_angles = fliplr(angles_rad(1, :) + pi/2);
%     subs_idx = new_angles > pi/2;
%     new_angles(subs_idx) = fliplr(new_angles(~subs_idx));
%     sin_angles = sin(new_angles);
    
    % Set flag, then wait max timeOut secs to receive data
    flag.Data = true;
    pause(1);
    send(pub_flag, flag);
    pause(1);
    msg = receive(depth_scan, timeOut);
    pause(1);
    
    d = msg.Data'/1000;
    
    % Do I really need to process? Maybe send raw, see receive_scan in Isis
    % Process data
%     range = z2range(d, sin_angles, quadrants);
%     sub_idx = (range > 0.28) & (range < 4);
    % Output processed values
%     z = [range(sub_idx); angles(sub_idx)];
    
    
    % NEW METHOD! #########################################################
    
    % Construct K (camera matrix)
    focal = 0.5 * depth_width / tan(0.5 * deg2rad(HFOV));
    K = [focal, 0, depth_width * 0.5; 0, focal, depth_height * 0.5; 0, 0, 1];
%     Sd = [depth_height, depth_width];
    iY = repmat(depth_width/2, 1, depth_width);
    iX = 1:depth_width;
    fullScan = [];
    l = depth_width;
    for j=0:quadrants-1
        start = j*l + 1;
        
        % First formula
    %     depth = [NaN(360, 1280); dT(start:((start-1)+l)); NaN(359, 1280)];
    %     [X,Y] = meshgrid(1:Sd(2),1:Sd(1));
    %     X = X - K(1,3) + 0.5;
    %     Y = Y - K(2,3) + 0.5;
    %     XDf = depth/K(1,1);
    %     YDf = depth/K(2,2);
    %     X = X .* XDf;
    %     Y = Y .* YDf;
    %     XY = cat(3,X,Y);
    %     cloud = cat(3,XY,depth);
    %     cloud = reshape(cloud,[],3) / 1000.0;
    
        % Second formula
        y = (iY - K(2, 3)) / K(2, 2);
        x = (iX - K(1, 3)) / K(1, 1);

        pX = d(start:((start-1)+l)) .* x;
        pY = d(start:((start-1)+l)) .* y;
        pZ = d(start:((start-1)+l));

        cloud = [pX; pY; pZ]';
        % Rotates each quadrant 90 degrees
        tformZYX = eul2tform([0, (j)*deg2rad(HFOV), 0]);
        tform = rigid3d(tformZYX);
        xyz_t = transformPointsForward(tform, cloud);
        fullScan = cat(1, fullScan, xyz_t);
    end
    % Rotate all points -90 degrees
    tformZYX = eul2tform([0, deg2rad(-90), 0]);
    tform = rigid3d(tformZYX);
    fullScan_r = transformPointsForward(tform, fullScan);
    [phi, r] = cart2pol(fullScan_r(:, 1), fullScan_r(:, 3));
    
    z = [r, phi]';
    z = z(:, r > 0);
end