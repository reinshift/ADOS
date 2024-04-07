function [intersected,point] = intersectCuboid(infoCuboid, rayStart, rayEnd)
%infoCylinder:[x y z  l w h  0 0 0]
x0 = rayStart(1);
y0 = rayStart(2);
z0 = rayStart(3);
x1 = rayEnd(1);
y1 = rayEnd(2);
z1 = rayEnd(3);
cx = infoCuboid(1);
cy = infoCuboid(2);
cz = infoCuboid(3);
l = infoCuboid(4);
w = infoCuboid(5);
h = infoCuboid(6);
P0 = [x0, y0, z0];
P1 = [x1, y1, z1];
direction = P1 - P0;

minCorner = [cx - l/2, cy - w/2, cz - h/2];
maxCorner = [cx + l/2, cy + w/2, cz + h/2];

planes = {
    [1, 0, 0, -maxCorner(1)], [1, 0, 0, -minCorner(1)], ... % x = max, x = min
    [0, 1, 0, -maxCorner(2)], [0, 1, 0, -minCorner(2)], ... % y = max, y = min
    [0, 0, 1, -maxCorner(3)], [0, 0, 1, -minCorner(3)]  ... % z = max, z = min
    };

intersectionPoints = [];
for i = 1:length(planes)
    plane = planes{i};
    t = -(dot(plane(1:3), P0) + plane(4)) / dot(plane(1:3), direction);
    if t >= 0 && t <= 1
        point = P0 + t * direction;
        if point(1) >= minCorner(1) && point(1) <= maxCorner(1) && ...
                point(2) >= minCorner(2) && point(2) <= maxCorner(2) && ...
                point(3) >= minCorner(3) && point(3) <= maxCorner(3)
            intersectionPoints = [intersectionPoints; point];
        end
    end
end
if  ~isempty(intersectionPoints)
    intersected = true;
    d = norm(intersectionPoints(1,:)-rayStart); point = intersectionPoints(1,:);
    for i = 2:size(intersectionPoints,1)
        if norm(intersectionPoints(i,:)-rayStart) < d
            d = norm(intersectionPoints(i,:)-rayStart);
            point = intersectionPoints(i,:);
        end
    end
else
    intersected = false;
    point = [];
end

end
