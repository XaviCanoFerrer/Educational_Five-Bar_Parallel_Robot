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

rpx = 3*xstd;
rpy = 3*ystd;

int = 0:pi/50:2*pi;
xem =  xstd * cos(int);
yem = ystd * sin(int);
plot(xem, yem, 'g','LineWidth',2);

hold on

int = 0:pi/50:2*pi;
xrp =  rpx * cos(int);
yrp = rpy * sin(int);
plot(xrp, yrp,'c','LineWidth',2);

title('Repeatability')
xlabel('x(mm)')
ylabel('y(mm)')
axis([-0.10 0.10 -0.2 0.2])
legend('Average error','Repeteability')

