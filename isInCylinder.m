function inside = isInCylinder(info,rayStart)
% info = [x y z x1 y1 z1 r]
    x0 = info(1);
    y0 = info(2);
    z0 = info(3);
    h = info(6) - z0;
    r = info(7);
    x = rayStart(1);
    y = rayStart(2);
    z = rayStart(3);
    inBase = (x - x0)^2 + (y - y0)^2 <= r^2;

    inHeight = (z >= z0 && z <= z0 + h);
    
    inside = inBase && inHeight;
end