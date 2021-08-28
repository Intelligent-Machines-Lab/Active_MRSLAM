% Supposed to work with TaskStructure

classdef myPoseGraph < matlab.mixin.SetGet
    properties
        % Nodes
        node
        % Edges
        edge
    end
    methods
        % Constructor
        function obj = myPoseGraph(s)
            if ~isempty(s)
                Rt = s.hist_rt{1};
                for i=1:2:size(Rt, 1)
                    T = [Rt(i:i+1, 1:3); 0, 0, 1];
                    zij = [T(1:2, 3); atan2(T(2, 1), T(1, 1))];

                    edges.z = zij;
                    edges.Omega = eye(3);
                    edges.From = Rt(i, 4) - 1;
                    edges.To = Rt(i, 4);

                    obj.edge = cat(1, obj.edge, edges);
                end
                for i=1:size(s.poses, 2)
                    nodes.x = s.poses(:, i);
                    nodes.id = i;

                    obj.node = cat(1, obj.node, nodes);
                end
            end
            
        end
        
        function add_node(obj, pose)
            nodes.x = pose;
            nodes.id = size(obj.node, 1) + 1;
            
            obj.node = cat(1, obj.node, nodes);
        end
        function add_node_new(obj, pose, id)
            nodes.x = pose;
            nodes.id = id;
            
            obj.node = cat(1, obj.node, nodes);
        end
        
        function add_edge(obj, id1, id2, omega)
            xi = obj.node(id1).x;
            xj = obj.node(id2).x;
            
            c = cos(xi(3));
            s = sin(xi(3));
            Xi = [c, -s, xi(1);
                  s,  c, xi(2);
                  0   0  1];
            c = cos(xj(3));
            s = sin(xj(3));
            Xj = [c, -s, xj(1);
                  s,  c, xj(2);
                  0   0  1];
            
            Z = Xi \ Xj;
            z = [Z(1:2, 3); atan2(Z(2, 1), Z(1, 1))];
            
            edges.z = z;
            edges.Omega = omega;
            edges.From = id1;
            edges.To = id2;

            obj.edge = cat(1, obj.edge, edges);
        end
        
        function add_edge_new(obj, z, id1, id2, omega)
%             c = cos(xi(3));
%             s = sin(xi(3));
%             Xi = [c, -s, xi(1);
%                   s,  c, xi(2);
%                   0   0  1];
%             c = cos(xj(3));
%             s = sin(xj(3));
%             Xj = [c, -s, xj(1);
%                   s,  c, xj(2);
%                   0   0  1];
%             
%             Z = Xi \ Xj;
%             z = [Z(1:2, 3); atan2(Z(2, 1), Z(1, 1))];
%             
            edges.z = z;
            edges.Omega = omega;
            edges.From = id1;
            edges.To = id2;

            obj.edge = cat(1, obj.edge, edges);
        end
        
        function update_nodes(obj, new_poses)
            for i=1:size(new_poses, 2)
                obj.node(i).x = new_poses(:, i);
            end
        end
        
        
        
        function viz_graph(obj)
            figure;
            ax = gca;
            hold(ax, 'on');
            
            for i=1:size([obj.edge.From], 2)
                obj.plot_edge(ax, i);
            end
            grid(ax, 'on');
            axis(ax, 'equal');
            title(ax, 'Pose Graph');
        end
        function plot_edge(obj, ax, idx)
            id1 = obj.edge(idx).From;
            id2 = obj.edge(idx).To;
            
            p1 = obj.node([obj.node.id] == id1).x;
            p2 = obj.node([obj.node.id] == id2).x;
            
            hold(ax, 'on');
            plot(ax, [p1(1), p2(1)], [p1(2), p2(2)], 'o-', 'Color', 'b', 'LineWidth', 2, 'MarkerSize', 3, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r')
        end
    end
end