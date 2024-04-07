function [intersected,point] = intersectCylinder(infoCylinder, rayStart, rayEnd)
intersected = false; point = [];
%infoCylinder:[x y z  x y z+h  r]
x0 = rayStart(1); y0 = rayStart(2); z0 = rayStart(3);
x1 = rayEnd(1); y1 = rayEnd(2); z1 = rayEnd(3);
x2 = infoCylinder(1); y2 = infoCylinder(2);
% h = infoCylinder(6);
r = infoCylinder(7);
a = (x1 - x0)^2 + (y1 - y0)^2;
b = 2 * ((x1 - x0) * (x0 - x2) + (y1 - y0) * (y0 - y2));
c = (x0 - x2)^2 + (y0 - y2)^2 - r^2;
delta = b^2 - 4 * a * c;
if delta < 0
    intersected = false; point = [];
    return;
elseif delta == 0
    intersected = true;
    t = -b / (2 * a);
    if t > 1 || t < 0
        intersected = false;
        return;
    else
        x = x0 + t * (x1 - x0);
        y = y0 + t * (y1 - y0);
        z = z0 + t * (z1 - z0);
        point = [x,y,z];
    end
else
    intersected = true;
    t1 = (-b - sqrt(delta)) / (2 * a);
    t2 = (-b + sqrt(delta)) / (2 * a);
    if t1 >= 0 && t1 <= 1 
        a1 = x0 + t1 * (x1 - x0);
        b1 = y0 + t1 * (y1 - y0);
        c1 = z0 + t1 * (z1 - z0);
        d1 = sqrt((a1 - x0)^2 + (b1 - y0)^2 + (c1 - z0)^2);
    else 
        a1 = NaN; b1 = NaN; c1 = NaN; d1 = Inf;
    end
    if t2 >= 0 && t2 <= 1
        a2 = x0 + t2 * (x1 - x0);
        b2 = y0 + t2 * (y1 - y0);
        c2 = z0 + t2 * (z1 - z0);
        d2 = sqrt((a2 - x0)^2 + (b2 - y0)^2 + (c2 - z0)^2);
    else
        a2 = NaN; b2 = NaN; c2 = NaN; d2 = Inf; % 用无穷大表示无效
    end
    if d1 == Inf && d2 == Inf
        intersected = false; point = [];
        return;
    elseif d1 < d2 % 交点1离起点近
        point = [a1 b1 c1];
    else % 交点2离起点近
        point = [a2 b2 c2];
    end
end

end