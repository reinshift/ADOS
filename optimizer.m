function F = optimizer(X)
%% obstacles initializing
o1 = Obstacle('cylinder',[30 15 0  30 15 50 5]);
o2 = Obstacle('cylinder',[50 25 0  50 25 50 5]);
o3 = Obstacle('cylinder',[30 35 0  30 35 50 5]);
o4 = Obstacle('cylinder',[50 45 0  50 45 50 5]);
o5 = Obstacle('cylinder',[30 55 0  30 55 50 5]);
o6 = Obstacle('cylinder',[50 65 0  50 65 50 5]);
o7 = Obstacle('cylinder',[30 75 0  30 75 50 5]);
o8 = Obstacle('cylinder',[50 85 0  50 85 50 5]);
o9 = Obstacle('cuboid',[50 50 -5  100 100 10  0 0 0]);
o10 = Obstacle('cylinder',[70 15 0  70 15 50 5]);
o11 = Obstacle('cylinder',[70 35 0  70 35 50 5]);
o12 = Obstacle('cylinder',[70 55 0  70 55 50 5]);
o13 = Obstacle('cylinder',[70 75 0  70 75 50 5]);
Obstacles = [o1 o2 o3 o4 o5 o6 o7 o8 o10 o11 o12 o13];

%% Goal function constructing
Tmax = 50;% test time
dt = 0.05;% diff t
N = 5; % swarm's size
goal = [80;75;40];
P_saclm = [X(1) X(2) X(3) X(4) X(5)]; % dim = 5,every P_sac denotes a population which participate in searching.
R = 0.5; % collision radius
% optimizing parameters

times = 3;
F = 0;
for k = 1:times
phi_vel = 0;
phi_corr = 0;
phi_u2u = 0;
swarm = [];
    for i = 1:N
        swarm = [swarm Drone([5+10*rand;5+10*rand;40],[0;0;0],6,P_saclm,goal)];
        if swarm(i).position == [NaN;NaN;NaN]
            swarm(i) = Drone([randi([5,15]);randi([5,15]);40],[0;0;0],6,P_saclm,goal);
        end
    end

    N_coll = zeros(1,N); % to account times of collision.
    N_arrive = zeros(1,N); % to account numbers of arrival
    for t = 1:dt:Tmax
        for i = 1:N
            swarm(i).updateV(swarm,Obstacles);
            swarm(i).updateP(dt);
            phi_vel = phi_vel + dt*norm(swarm(i).velocity);
            phi_corr = phi_corr + dt*swarm(i).cal_cosAngle(swarm);
            phi_u2u = phi_u2u + dt*swarm(i).calPhiU2u(swarm,R);
            if swarm(i).isInsideObs == true
                N_coll(i) = 1;
                continue;
            else
                N_coll(i) = 0;
            end
            if norm(swarm(i).position - swarm(i).p_goal) <= 4.9
                N_arrive(i) = 1;
            else
                N_arrive(i) = 0;
            end
        end
        if N_arrive + N_coll == ones(1,N)
            break;
        end
    end
    phi_vel = phi_vel/((t-1)*N*swarm(1).vel_0);
    phi_corr = phi_corr/((t-1)*N);
    phi_u2o = sum(N_coll);
    phi_u2u = phi_u2u/((t-1)*N*(N-1));
    phi_arrive = sum(N_arrive)/N;
    F = F + 1/abs(phi_vel-1)*(phi_corr + 1)/2*heaviside(1-phi_u2u)*heaviside(1-phi_u2o)*phi_arrive;
end
    F = (-1)*F/times;
end