clear;clc;close all;
%% initializing
o1 = Obstacle('cylinder',[40 80 0  40 80 50 5]);
o2 = Obstacle('cylinder',[40 60 0  40 60 50 5]);
o3 = Obstacle('cuboid',[80 80 25  5 40 50  0 0 0]);
o4 = Obstacle('cuboid',[20 20 25  5 40 50  0 0 0]);
o5 = Obstacle('cylinder',[60 40 0  60 40 50 5]);
o6 = Obstacle('cylinder',[60 20 0  60 20 50 5]);
o7 = Obstacle('cuboid',[50 -5.5 50  100 10 100  0 0 0]);
o8 = Obstacle('cuboid',[50 105.5 50  100 10 100  0 0 0]);
o9 = Obstacle('cuboid',[50 50 -5  100 100 10  0 0 0]);
Obstacles = [o1 o2 o3 o4 o5 o6 o7 o8];
Tmax = 30;% test time
dt = 0.05;% diff t
N = 5; % swarm's size
swarm = [];
P_saclm = [7.0789 1.8849 0.01 0.00032157 1.1]; %[ps pa pc lambda mu]
goal = [80;40;40];% destination
for i = 1:N
    swarm = [swarm Drone([randi([0,10]);randi([5,15]);40],[0;0;0],6,P_saclm,goal)];
    if swarm(i).position == [NaN;NaN;NaN]
        swarm(i) = Drone([randi([0,10]);randi([5,15]);40],[0;0;0],6,P_saclm,goal);
    end
end
%% data diary
Aver_d = zeros(1,Tmax/dt+1); % Average distance between UAVs
Aver_v = zeros(1,Tmax/dt+1); % Average velocity of UAVs
min_d = zeros(1,Tmax/dt+1); % Minimum distance between UAVs
max_d = zeros(1,Tmax/dt+1); % Maximum distance between UAVs
historyTrajectory = zeros(3,Tmax/dt+1,N); % to save history trajectory
historyVel = zeros(3,Tmax/dt+1,N); % to save history velocity
min_v = zeros(1,Tmax/dt+1); % Minimum velocity of UAVs
max_v = zeros(1,Tmax/dt+1); % Maximum velocity of UAVs

%% draw obstacles
simulation_fig = figure('Color','w'); hold on;
drawCylinder(o1.Size,'FaceColor',[0.97,0.98,0.78]);
drawCylinder(o2.Size,'FaceColor',[0.97,0.98,0.78]);
drawCuboid(o3.Size,'FaceColor',[0.97,0.98,0.78]);
drawCuboid(o4.Size,'FaceColor',[0.97,0.98,0.78]);
drawCylinder(o5.Size,'FaceColor',[0.97,0.98,0.78]);
drawCylinder(o6.Size,'FaceColor',[0.97,0.98,0.78]);
drawCuboid(o7.Size,'FaceColor',[0.97,0.98,0.78]);
drawCuboid(o8.Size,'FaceColor',[0.97,0.98,0.78]);
drawCuboid(o9.Size,'FaceColor',[1,1,1]);
plot3(swarm(1).p_goal(1),swarm(1).p_goal(2),swarm(1).p_goal(3),'r*')
axis equal;
grid on;
view([-0.2 80.1])
% view([-31.7 87.0])
axis([0 100 0 55 0 100]);
set(gcf, 'Position', [100, 100, 800, 600]);
lighting gouraud;
light;
scatterHandles = gobjects(N, 1);
scatterColors = hsv(N); % save color
box on;
rotate3d on;

sum_coll = zeros(1,N); % to account times of collision.
sum_arrive = zeros(1,N); % to account numbers of arrival
for t = 1:dt:Tmax
    for i = 1:N
        if isvalid(scatterHandles(i))
            delete(scatterHandles(i));
        end
        scatterHandles(i) = scatter3(swarm(i).position(1),swarm(i).position(2),swarm(i).position(3),'filled','MarkerEdgeColor', 'k', 'LineWidth', 1);
        scatterHandles(i).CData = scatterColors(i, :);
        swarm(i).updateV(swarm,Obstacles);
        swarm(i).updateP(dt);
        current_t = floor((t-1)/dt+1.01);
        historyTrajectory(:,current_t,i) = swarm(i).position;
        historyVel(:,current_t,i) = swarm(i).velocity;
    end

    pause(0.05)
    drawnow
    % if t == 1 || t == 100*dt || t == 150*dt || t == 175*dt || t == 200*dt || t == 225*dt || t == 250*dt || t == 300*dt || t == 350*dt || t == 400*dt
    %     print([['运行于第' num2str(t) 's'],'.eps'],'-depsc','-r600');
    %     % saveas(simulation_fig,['D:\Study\导师任务\发表论文\23_11_30\runtime\cuboid\' ['运行于第' num2str(t) 's'] '.png']);
    % end
    for i = 1:N
        if swarm(i).isInsideObs == true
            sum_coll(i) = 1;
            continue;
        else
            sum_coll(i) = 0;
        end
        if norm(swarm(i).position - swarm(i).p_goal) <= 4.9
            sum_arrive(i) = 1;
        else
            sum_arrive(i) = 0;
        end
    end

    if sum(sum_arrive) == N
        disp('Mission completed, all UAVs reached their goal.');
        break;
    elseif sum_arrive + sum_coll == ones(1,N)
        disp('Mission failed, some UAVs have hit the wall.');
        break;
    end
end
% print([['运行于第' num2str(t) 's'],'.eps'],'-depsc','-r600');
hold off;
%% plot trajectory
figure;
hold on;
for i = 1:N
    x = historyTrajectory(1,1:current_t,i);
    y = historyTrajectory(2,1:current_t,i);
    z = historyTrajectory(3,1:current_t,i);
    plot3(x,y,z,'LineWidth',1,'DisplayName', ['UAV' num2str(i)]);
end
legend('Location','southeast');
title('Route Planning of UAVs');
plot3(swarm(1).p_goal(1),swarm(1).p_goal(2),swarm(1).p_goal(3),'r*','DisplayName','Goal Point');
drawCylinder(o1.Size,'FaceColor',[0.97,0.98,0.78],'HandleVisibility','off');
drawCylinder(o2.Size,'FaceColor',[0.97,0.98,0.78],'HandleVisibility','off');
drawCuboid(o3.Size,'FaceColor',[0.97,0.98,0.78],'HandleVisibility','off');
drawCuboid(o4.Size,'FaceColor',[0.97,0.98,0.78],'HandleVisibility','off');
drawCylinder(o5.Size,'FaceColor',[0.97,0.98,0.78],'HandleVisibility','off');
drawCylinder(o6.Size,'FaceColor',[0.97,0.98,0.78],'HandleVisibility','off');
drawCuboid(o7.Size,'FaceColor',[0.97,0.98,0.78],'HandleVisibility','off');
drawCuboid(o8.Size,'FaceColor',[0.97,0.98,0.78],'HandleVisibility','off');
% drawCuboid(o9.Size,'FaceColor',[1,1,1],'HandleVisibility','off');
axis equal;
axis([0 100 0 55 0 100]);
box on;
light;
hold off;
% print('Route_Planning_of_UAVs.eps','-depsc','-r600');

%% plot other parameters
for t = 1:length(Aver_d)
    dmin = 100; vmin = 100;
    dmax = 0; vmax = 0;
    for i = 1:N
        for j = i+1:N
            d = norm(historyTrajectory(:,t,i)-historyTrajectory(:,t,j));% d(i,j)
            Aver_d(t) = Aver_d(t) + d;
            if d < dmin
                dmin = d;
            end
            if d > dmax
                dmax = d;
            end
        end
        v = norm(historyVel(:,t,i));
        Aver_v(t) = Aver_v(t) + v;
        if v < vmin
            vmin = v;
        end
        if v > vmax
            vmax = v;
        end
    end
    min_d(t) = dmin;
    max_d(t) = dmax;
    min_v(t) = vmin;
    max_v(t) = vmax;
end
Aver_d = Aver_d/nchoosek(N,2);
Aver_v = Aver_v/N;
% plot distance
figure('Position', [100, 100, 1200, 300]);
hold on;
title('distance between UAVs');
plot(0:dt:(current_t-1)*dt,Aver_d(1:current_t),'b','LineWidth',2,'DisplayName','d_{Average}');
x = 0:dt:(current_t-1)*dt;
min_d_values = min_d(1:current_t);
max_d_values = max_d(1:current_t);
fill([x, fliplr(x)], [min_d_values, fliplr(max_d_values)], 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Range');
% plot(0:dt:(current_t-1)*dt,min_d(1:current_t),'b','LineWidth',1,'DisplayName','d_{min}');
% plot(0:dt:(current_t-1)*dt,max_d(1:current_t),'b','LineWidth',1,'DisplayName','d_{max}');
plot(0:dt:(current_t-1)*dt,swarm(1).rs*ones(1,current_t),'k--','LineWidth',1,'DisplayName','d_{safe}');
plot(0:dt:(current_t-1)*dt,0.5*ones(1,current_t),'r--','LineWidth',1,'DisplayName','d_{warn}');
legend('Location','northwest');
xlim([0 (current_t-1)*dt])
xlabel('Time/s');
ylabel('distance/m');
grid on;
box on;
hold off;
% print('distance_between_UAVs_cuboid.eps', '-depsc');

% plot velocity
figure('Position', [200, 200, 1200, 300]);
hold on;
title('velocity of UAVs');
plot(0:dt:(current_t-1)*dt,Aver_v(1:current_t),'b','LineWidth',2,'DisplayName','V_{Average}');
min_v_values = min_v(1:current_t);
max_v_values = max_v(1:current_t);
fill([x, fliplr(x)], [min_v_values, fliplr(max_v_values)], 'b', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Range');
% plot(0:dt:(current_t-1)*dt,min_v(1:current_t),'b','LineWidth',1,'DisplayName','V_{min}');
% plot(0:dt:(current_t-1)*dt,max_v(1:current_t),'b','LineWidth',1,'DisplayName','V_{max}');
plot(0:dt:(current_t-1)*dt,swarm(1).vel_0*ones(1,current_t),'k--','LineWidth',1,'DisplayName','V_{0}');
legend('Location','northeast');
xlim([0 (current_t-1)*dt])
ylim([0 12.5])
xlabel('Time/s');
ylabel('velocity/(m \cdot s^{-1})');
grid on;
box on;
hold off;
% print('velocity_of_UAVs_cuboid.eps', '-depsc');