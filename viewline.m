clc;clear;
%% parameters initializing
% cylinder
r_cylinder = 10;
h_cylinder = 50;
p0_cylinder = [10,10,0];
p1_cylinder = p0_cylinder + h_cylinder*[0 0 1];
% circle
center = [0,0,40];
radius = 10;
L = 12;
theta = 2*pi/L;

%% draw cylinder
figure;
cylinder = [p0_cylinder p1_cylinder r_cylinder];
drawCylinder(cylinder,'FaceColor',[0.97,0.98,0.78]);
light;

%% draw circle
hold on; 
drawCircle3d([center radius 0 0], 'LineWidth', 2, 'Color', [0,0,0]);
% axis equal;
axis([-10 20 -10 20 0 60]);

%% draw lines
for i = 1:L
    line = [center cos(i*theta) sin(i*theta) 0];
    points = intersectLineCylinder(line, cylinder);
    nrow = size(points,1);
    if  nrow == 1
        if norm(points-center) + norm(points-(center+radius*[cos(i*theta) sin(i*theta) 0])) > radius
            drawEdge3d([center;center+radius*[cos(i*theta) sin(i*theta) 0]],'k');
        else
            % drawLine3d(line,'Color','k');
            drawEdge3d([center;points(1,:)],'r');
        end
    elseif nrow == 2
        if norm(center'-points(1,:)')<norm(center-points(2,:))
            if norm(points(1,:)-center) + norm(points(1,:)-(center+radius*[cos(i*theta) sin(i*theta) 0])) > radius
                drawEdge3d([center;center+radius*[cos(i*theta) sin(i*theta) 0]],'k');
            else
                drawEdge3d([center;points(1,:)],'r');
            end
        else
            if norm(points(2,:)-center) + norm(points(2,:)-(center+radius*[cos(i*theta) sin(i*theta) 0])) > radius
                drawEdge3d([center;center+radius*[cos(i*theta) sin(i*theta) 0]],'k');
            else
                drawEdge3d([center;points(1,:)],'r');
            end
        end
    else
        drawEdge3d([center;center+radius*[cos(i*theta) sin(i*theta) 0]],'k')
    end
end
hold off;