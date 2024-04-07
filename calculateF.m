function F = calculateF(rays,v_goal,rs)
    N = size(rays,1);
    F_dire = zeros(1,N);
    F_dist = zeros(1,N);
    for i = 1:N
        F_dire(i) = exp(rays(i,:)*v_goal/(2*norm(rays(i,:))*norm(v_goal))-1);
        % F_dist(i) = prod([sqrt(H((norm(rays(mod(i-3, N) + 1,:))-rs)/100)) ...
        %     H((norm(rays(mod(i-2, N) + 1,:))-rs)/100) ...
        %     (H((norm(rays(mod(i-1,N) + 1,:))-rs)/100))^2 ...
        %     H((norm(rays(mod(i,N) + 1,:))-rs)/100) ...
        %     sqrt(H((norm(rays(mod(i+1,N) + 1,:))-rs)/100)) ]);
        F_dist(i) = prod([H((norm(rays(mod(i-2, N) + 1,:))-rs)/norm(rays(mod(i-2, N) + 1,:)))^2 ...
            H((norm(rays(mod(i-1,N) + 1,:))-rs)/norm(rays(mod(i-1,N) + 1,:))) ...
            H((norm(rays(mod(i,N) + 1,:))-rs)/norm(rays(mod(i,N) + 1,:)))^2]);
    end
    F = F_dire.*(F_dist.^2);
end

% for i = 1:N
%     plot3(rays(i,1),rays(i,2),rays(i,3),'b+');
% end