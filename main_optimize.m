claer;clc;
%% optimizing with SSA
pop = 40; %population size
iter_Max = 100;
lb = [0 0 0.01 0 0];
ub = [10 2 1 1 3];
dim = 5;
[fMin,bestX,Convergence_curve] = SSA(pop,iter_Max,lb,ub,dim,@optimizer);

%% plot Convergence Curve
CNT=35;
k=round(linspace(1,iter_Max,CNT));
iter=1:1:iter_Max;
figure;
semilogy(iter(k),Convergence_curve(k),'g->','linewidth',1);
hold on
grid on;
title('收敛曲线')
xlabel('迭代次数');
ylabel('适应度值');
box on
legend('SSA')
set (gcf,'position', [300,300,800,320])