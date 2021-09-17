clc 
clear all

%% FIVE-BAR PARALLEL ROBOT REPEATABILITY TEST


%Measured points

x = [1.24, 1.27, 1.25, 1.29, 1.31, 1.31, 1.31, 1.31, 1.33, 1.29, 1.31, 1.31, 1.31, 1.30, 1.30, 1.30, 1.29, 1.28];
y = [0.39, 0.32, 0.39, 0.39, 0.36, 0.38, 0.45, 0.48, 0.39, 0.41, 0.43, 0.48, 0.45, 0.47, 0.46, 0.49, 0.49, 0.39];

xm = mean(x);
ym = mean(y);

xstd = std(x);
ystd = std(y);


for i=1:1:18
    Lj(i)=sqrt((x(i)-xm)^2+(y(i)-ym)^2);
end

Lm = mean(Lj);
Ls = std(Lj);

RP = Lm+3*Ls;

int = 0:pi/50:2*pi;
xem = Lm* cos(int);
yem = Lm* sin(int);
plot(xem, yem, 'g','LineWidth',2);

hold on

xrp =  RP * cos(int);
yrp = RP * sin(int);
plot(xrp, yrp,'c','LineWidth',2);

title('Repeatability')
xlabel('x(mm)')
ylabel('y(mm)')
axis([-0.15 0.15 -0.15 0.15])
axis equal
legend('Average error','Repeteability')