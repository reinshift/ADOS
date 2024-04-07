function inside = isInCuboid(info,rayStart)
% info = [x y z l w h 0 0 0]
x = rayStart(1);
y = rayStart(2);
z = rayStart(3);
x0 = info(1);
y0 = info(2);
z0 = info(3);
l = info(4);
w = info(5);
h = info(6);
inside_x = (x >= x0-l/2 && x <= x0+l/2);
inside_y = (y >= y0-w/2 && y <= y0+w/2);
inside_z = (z >= z0-h/2 && z <= z0+h/2);
inside = inside_x && inside_y && inside_z;
end