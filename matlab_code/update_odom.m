% Using kinematic equations to generate relative pose estimations
% ATTENTION: Params set for ISIS
function [pose_new, traj] = update_odom(pose_old, u, dt)
    model = differentialDriveKinematics();
    model.VehicleInputs = 'VehicleSpeedHeadingRate';
    model.TrackWidth = 0.27;
    tspan = 0:0.05:dt;
    [~, y] = ode45(@(t, y)derivative(model, y, u), tspan, pose_old);
    pose_new = y(end, :)';
    if nargout > 1
        traj = y';
    end
end