close all;
clear all;

x_min = -4;
x_max = 4;
y_min = -4;
y_max = 4;

% 2024-10-11:
dis = [3.043548583984375, 2.33171284198761, 2.6561365127563477];
anchorEst = [2.268828511837818, -1.1268009384288105];
anchorTruth = [2.1064062733943754, -1.2588366872347918];

d = 0.6;
d_wu = 0.016;
tagX = [-(d * sqrt(3) / 2 - d_wu), d_wu, d_wu];
tagY = [0, -d/2, d/2];

figure(1);
hold on;

for i = 1:3
  plot(tagX(i), tagY(i), 'x', 'color', lines()(i,:));
end
for i = 1:3
  drawCircle(tagX(i), tagY(i), dis(i), 'color', lines()(i,:));
end
plot(anchorEst(1), anchorEst(2), '+k', 'markersize', 10);
plot(anchorTruth(1), anchorTruth(2), 'x', 'color', [0, 0.5, 0], 'markersize', 10);

xlim([x_min x_max]);
ylim([y_min y_max]);
%legend({'', '', 'Tags', '', '', 'Distances', 'g', 'h'}, 'orientation', 'horizontal', 'location', 'southoutside', 'numcolumns', '3');
axis("equal");
title("Position Determination of an UWB Anchor");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
grid on;
print -depslatex ../../../../ma-thesis/pics/eps/anchorPosDeterm.eps