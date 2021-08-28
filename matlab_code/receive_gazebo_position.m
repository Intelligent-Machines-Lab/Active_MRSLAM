% Returns the pose of a gazebo model
% input: topic = name of the topic, usually is 'gazebo/model_states'
%        modelname = model name inside topic, check 'rostopic echo topic'
% output: pose = (x,y,th)' of the model

function [pose] = receive_gazebo_position(topic, modelname)
    msg = receive(topic);
    idx = 0;
    for i=1:length(msg.Name)
        if strcmp(msg.Name{i}, modelname)
            idx = i;
        end
    end
    if idx
        pos = msg.Pose(idx);
        q = [pos.Orientation.W, pos.Orientation.X, pos.Orientation.Y, pos.Orientation.Z];
        th = quat2eul(q); %ZYX sequence
        pose = [pos.Position.X; pos.Position.Y; th(1)]; % theta in radians
    else
        disp('ModelName not found in Gazebo topic.');
        pose = [];
    end
end