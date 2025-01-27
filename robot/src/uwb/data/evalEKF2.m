close all;
clear all;

M = colorgradient([0,0.5,0; 0.9,0.9,0; 0.975,0.25,0; 1,0,0], [1,1,0.15], 500);
or = [1, 0.6, 0];
gr = [0,0.5,0];

max_err = 1;

x_min = -1;
x_max = 12;
y_min = -4;
y_max = 5;

data0_0001 = csvread("trajectories/2024-12-18/0_0001.csv");
data0_01 = csvread("trajectories/2024-12-18/0_01.csv");
data1 = csvread("trajectories/2024-12-18/1.csv");
data100 = csvread("trajectories/2024-12-18/100.csv");
data1_1 = csvread("trajectories/2024-12-18/1_1.csv");
data1_5 = csvread("trajectories/2024-12-18/1_5.csv");
data1_50 = csvread("trajectories/2024-12-18/1_50.csv");
data1_500 = csvread("trajectories/2024-12-18/1_500.csv");

%ekf_state: 1-11
%ekf_var: 12-22
%uwb_robot_pos: 23-25
%uwb_anchor_pos: 26-33
%odom: 34-36
%hector: 37-39

figure(1);
hold on;

%plot lines:
plot(data1(:,34), data1(:,35), 'b');
plot(data1(:,37), data1(:,38), 'color', or);
plot(data0_0001(:,1), data0_0001(:,2));
plot(data0_01(:,1), data0_01(:,2));
plot(data1(:,1), data1(:,2));
plot(data100(:,1), data100(:,2));

xlim([x_min x_max]);
ylim([y_min y_max]);
axis("equal");
title("Trajectories for varying $\sigma_{anPos}$");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
grid on;
legend({"Wheel Odometry", "Hector SLAM", "$\sigma_{anPos}^2 = 0.0001$", "$\sigma_{anPos}^2 = 0.01$", "$\sigma_{anPos}^2 = 1$", "$\sigma_{anPos}^2 = 100$"}, 'location', 'northeastoutside');
%print -depsc trajectories/2024-12-18/traAnPos.eps
%print -depslatex ../../../../ma-thesis/pics/eps/traAnPos2024_12_18.eps

figure(2);
hold on;

%plot lines:
plot(data1(:,34), data1(:,35), 'b');
plot(data1(:,37), data1(:,38), 'color', or);
plot(data1(:,1), data1(:,2));
plot(data1_1(:,1), data1_1(:,2));
plot(data1_5(:,1), data1_5(:,2));
plot(data1_50(:,1), data1_50(:,2));
plot(data1_500(:,1), data1_500(:,2));

xlim([x_min x_max]);
ylim([y_min y_max]);
axis("equal");
title("Trajectories for varying $\sigma_{dis}$");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
grid on;
legend({"Wheel Odometry", "Hector SLAM", "Without", "$\sigma_{dis}^2 = 0.001$", "$\sigma_{dis}^2 = 0.005$", "$\sigma_{dis}^2 = 0.05$", "$\sigma_{dis}^2 = 0.5$"}, 'location', 'northeastoutside');
%print -depsc trajectories/2024-12-18/traDis.eps
%print -depslatex ../../../../ma-thesis/pics/eps/traDis2024_12_18.eps