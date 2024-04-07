clear;
x = linspace(-1,1,100);
f = zeros(1,length(x));
h = zeros(1,length(x));
for i = 1:length(x)
    h(i) = H(x(i));
    f(i) = exp(x(i)/2-1);
end
figure;

subplot(1,2,1);
plot(x,h,'LineWidth',2)
axis equal;
axis([-1 1 -0.01 1.01]);
title('\Theta(x)');
grid on;

subplot(1,2,2);
plot(x,f,'LineWidth',2);
axis equal;
axis([-1 1 -0.01 1.01]);
title('exp(x/2-1)');
grid on;

print('function_images.eps','-depsc')