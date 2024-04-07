classdef Drone < handle
    properties
        position;
        velocity;
        vel_0; % a scalar quantity
        vel_s; % seperation
        vel_a; % alignment
        vel_c; % cohesion
        vel_obs; % avoid obstacle
        vel_goal; % velocity2goal
        % linear gain for sac (waiting for optimizing)
        ps;
        pa;
        pc;
        lambda;
        mu;
        p_goal;
        isInsideObs;
    end
    properties(Constant)
        % position of goal
        % allowance parameters
        rs = 5;
        ra = 10;
        va = 2;
        rc = 40;
        % constrictions for dynamics
        Vmax = 12; % m/s
        amax = 6; % m*s^(-2)
    end
    methods
        % construct function.
        function obj = Drone(pos,vel,v0,P,goal)
            obj.position = pos;
            obj.velocity = vel;
            obj.vel_0 = v0;
            obj.vel_s = zeros(3,1);
            obj.vel_a = zeros(3,1);
            obj.vel_c = zeros(3,1);
            obj.ps = P(1);
            obj.pa = P(2);
            obj.pc = P(3);
            obj.lambda = P(4);
            obj.mu = P(5);
            obj.p_goal = goal;
            obj.isInsideObs = false;
        end

        function obj = updateV_s(obj,swarm)
            obj.vel_s = zeros(3,1);
            for j = 1:numel(swarm)
                if swarm(j) ~= obj && swarm(j).isInsideObs == false
                    Dij = norm(swarm(j).position-obj.position);
                    if  Dij <= obj.rc
                        obj.vel_s = obj.vel_s + (obj.rs - Dij)...
                            *(obj.position-swarm(j).position)/Dij...
                            *heaviside(obj.rs - Dij);
                    end
                end
            end
            obj.vel_s = obj.ps*obj.vel_s;
        end

        function obj = updateV_a(obj,swarm)
            obj.vel_a = zeros(3,1);
            for j = 1:numel(swarm)
                if swarm(j) ~= obj && swarm(j).isInsideObs == false
                    Vij = norm(swarm(j).velocity - obj.velocity);
                    if Vij == 0
                        continue;
                    end
                    if  norm(swarm(j).position - obj.position)<= obj.ra
                        obj.vel_a = obj.vel_a + (obj.va - Vij)...
                            *(obj.velocity - swarm(j).velocity)/Vij...
                            *heaviside(obj.va - Vij);
                    end
                end
            end
            obj.vel_a = obj.pa*obj.vel_a;
        end

        function obj = updateV_c(obj,swarm)
            obj.vel_c = zeros(3,1);
            for j = 1:numel(swarm)
                if swarm(j) ~= obj && swarm(j).isInsideObs == false
                    Dij = norm(swarm(j).position - obj.position);
                    if  Dij <= obj.rc && Dij >= obj.rs
                        obj.vel_c = obj.vel_c + (Dij - obj.rc)...
                            *(obj.position-swarm(j).position)/Dij...
                            *heaviside(obj.rc - Dij);
                    end
                end
            end
            obj.vel_c = obj.pc*obj.vel_c;
        end

        function obj = updateV_goal(obj)
            obj.vel_goal = obj.vel_0*(obj.p_goal - obj.position)/...
                norm(obj.p_goal - obj.position);
        end

        function obj = updateV_obs(obj, obstacles)
            L = 12;
            theta = 2*pi/L;
            angle = 0:theta:(2*pi - theta);
            rays = zeros(L,3); % to save ray's end.
            rayLength = obj.rc;
            for i = 1:L
                rayDir = [cos(angle(i)), sin(angle(i)), 0]; % ray's direction
                rays(i,:) = (obj.position)' + rayLength * rayDir; % earth coordinate
                for j = 1:numel(obstacles)
                    if obstacles(j).isInObstacle((obj.position)')% if collided
                        obj.vel_goal = zeros(3,1);
                        obj.vel_s = zeros(3,1);
                        obj.vel_a = zeros(3,1);
                        obj.vel_c = zeros(3,1);
                        obj.vel_obs = zeros(3,1);
                        obj.isInsideObs = true;
                        return;
                    end
                    [intersected,point] = obstacles(j).intersectWithRay((obj.position)', rays(i,:));
                    if intersected && norm(rays(i,:) - (obj.position)') > norm(point - (obj.position)')
                        rays(i,:) = point;
                    end
                end
                rays(i,:) = rays(i,:)-(obj.position)';% at this moment used to save ray's vector
            end
        F = calculateF(rays,obj.vel_goal,obj.rs);

        % introduce a new rule
        % corner = zeros(1,L);
        % for i = 1:L
        %     inn_product = rays(i,:)*obj.vel_goal;
        %     if inn_product == 0
        %         corner(i) = 0;
        %     else
        %         corner(i) = inn_product/(norm(rays(i,:))*norm(obj.vel_goal));
        %     end
        % end
        % [~, min_corner_index] = min(corner);
        % rayDist = zeros(1,L);
        % for i = 1:L
        %     rayDist(i) = norm(rays(i,:));
        % end
        % % randomly decide whether to try a bad direction
        % p = F(min_corner_index)/sum(F);
        % r = randi([0,1]);
        % if min(rayDist) <= 5 && norm(obj.velocity) ~= 0 && r < p
        %     F(min_corner_index) = F(min_corner_index) + max(F);
        % end

        % [~, min_F_index] = min(F);

        % if min_F_index == (mod(max_corner_index-1,L) + 1) && norm(obj.velocity) ~= 0
        %     k = mod(max_corner_index-1,L) + 1;
        %     Fmax = max(F);
        %     F(mod(k-1+L/4,L) + 1) = F(mod(k-1+L/4,L) + 1) + Fmax;
        %     F(mod(k-1-L/4,L) + 1) = F(mod(k-1-L/4,L) + 1) + Fmax;
        % elseif min_F_index == (mod(max_corner_index-2,L) + 1) && norm(obj.velocity) ~= 0
        %     k = mod(max_corner_index-2,L) + 1;
        %     Fmax = max(F);
        %     F(mod(k-1+L/4,L) + 1) = F(mod(k-1+L/4,L) + 1) + Fmax;
        %     F(mod(k-1-L/4,L) + 1) = F(mod(k-1-L/4,L) + 1) + Fmax;
        % elseif min_F_index == (mod(max_corner_index,L) + 1) && norm(obj.velocity) ~= 0
        %     k = mod(max_corner_index,L) + 1;
        %     Fmax = max(F);
        %     F(mod(k-1+L/4,L) + 1) = F(mod(k-1+L/4,L) + 1) + Fmax;
        %     F(mod(k-1-L/4,L) + 1) = F(mod(k-1-L/4,L) + 1) + Fmax;
        % end
        index = find(F == max(F));
        lb = (rays(index,:))';
        obj.vel_obs = obj.vel_0*lb/norm(lb);
        end

        function obj = updateV(obj,swarm,obstacle)
            obj.updateV_s(swarm);
            obj.updateV_a(swarm);
            obj.updateV_c(swarm);
            obj.updateV_goal;
            obj.updateV_obs(obstacle);
            vd = obj.vel_s + obj.vel_a + obj.vel_c + obj.lambda*(obj.vel_goal) + obj.mu*(obj.vel_obs);
            if obj.isInsideObs == true || norm(vd) == 0
                obj.velocity = [0;0;0];
            else
                obj.velocity = vd/norm(vd)*min(norm(vd),obj.Vmax);
            end
        end

        function obj = updateP(obj,dt)
            obj.position = obj.position + obj.velocity * dt;
        end

        function cosAngle = cal_cosAngle(obj,swarm)
            cardNc = 0;
            cosAngle = 0;
            for j = 1:numel(swarm)
                if swarm(j) ~= obj && (swarm(j).isInsideObs == false) ...
                        && norm(swarm(j).position - obj.position)<=obj.rc
                    cardNc = cardNc + 1;
                    vi = norm(obj.velocity);
                    vj = norm(swarm(j).velocity);
                    if vi*vj == 0
                        cosAngle = 1;
                    else
                        cosAngle = cosAngle + dot(swarm(j).velocity,obj.velocity)/(norm(swarm(j).velocity)*norm(obj.velocity));
                    end
                end
            end
            if cardNc ~= 0
                cosAngle = cosAngle/cardNc;
            end
        end

        function phi_u2u = calPhiU2u(obj,swarm,R)
            phi_u2u = 0;
            for j = 1:numel(swarm)
                if swarm(j) ~= obj && (swarm(j).isInsideObs == false) ...
                        phi_u2u = phi_u2u + heaviside(norm(obj.position - swarm(j).position) - R);
                end
            end
        end
    end
end