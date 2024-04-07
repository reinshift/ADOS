classdef Obstacle < handle

    properties
        Type
        Size
    end

    methods
        % constructor
        function obj = Obstacle(type,size)
            obj.Type = type;
            if strcmp(obj.Type,'cylinder')
                obj.Size = zeros(1,7);
            elseif strcmp(obj.Type,'cuboid')
                obj.Size = zeros(1,9);
            end
            obj.Size = size;
            % if cylinder then: [x,y,z,x1,y1,z1,r];
            % if cuboid   then: [x,y,z,l,w,h,0,0,0];
        end
        
        % examing intersection
        function [intersected,point] = intersectWithRay(obj, rayStart, rayEnd)
            intersected = false;point = [];
            if strcmp(obj.Type,"cuboid")
                [intersected,point] = intersectCuboid(obj.Size,rayStart,rayEnd);
            elseif strcmp(obj.Type,"cylinder")
                [intersected,point] = intersectCylinder(obj.Size,rayStart,rayEnd);
            end
        end

        % examing isInObstacle
        function inside = isInObstacle(obj,rayStart)
            inside = false;
            if strcmp(obj.Type,"cuboid")
                inside = isInCuboid(obj.Size,rayStart);
            elseif strcmp(obj.Type,"cylinder")
                inside = isInCylinder(obj.Size,rayStart);
            end
        end

    end
end