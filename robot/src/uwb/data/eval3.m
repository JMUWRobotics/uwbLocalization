close all;
clear all;

num = 4;

x = [0, 5.44414953, 5.23606093, 2.53991642];
y = [0, 0, 5.35165199, 2.99224572];
x1 = [0, 5.44413492, 5.2362371, 2.54075866];
y1 = [0, 0, 5.35076848, 2.99155933];
x_real = [0, 5, 5, 2.5];
y_real = [0, 0, 5, 2.5];

figure(1);
hold on;
plot(x, y, 'rx');
plot(x1, y1, 'gx');
plot(x_real, y_real, 'bo');
axis([-1, 6, 0, 6], 'equal');
title("UWB anchor position reconstruction");
xlabel("x axis [m]");
ylabel("y axis [m]");
legend(["uwb reconstructed averaged 50 measurments"; "uwb reconstructed averaged 50 maps"; "real"], 'location', 'southoutside');
grid on;
#print("-depsc", [filename, ".eps"])