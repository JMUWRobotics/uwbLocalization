close all;
clear all;

num = 17;

for i = [1:num]
  data(i,:) = csvread(["distances/indoor3_4_21_5/distancesOut", num2str(i), "m.csv"]);
end

data = data.';

for i = [1:num]
  err(:,i) = data(:,i) - i;
end

mean(data)
std(data)

% figure(1);
% boxplot(data);
% title("Distances between UWB nodes");
% xlabel("Real Distance [m]");
% ylabel("UWB measured distance [m]");
% grid on;

% figure(2);
% boxplot(err);
% ylim([0 1])
% title("Distance error between UWB nodes");
% xlabel("Real Distance [m]");
% ylabel("Error of UWB measurment [m]");
% grid on;
% print -depsc distances/indoor3_4_21_5/distanceErrorindoor3_4_21_5.eps
% print -depslatex ../../../../ma-thesis/pics/eps/distanceErrorindoor3_4_21_5.eps