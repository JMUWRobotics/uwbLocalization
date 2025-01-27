close all;
clear all;

gr = [0, 0.5, 0]

x1 = [0, 5.05954874, 5.28110137, -0.12604829];
y1 = [0, 0, 5.19671931, 5.31523734];
x2 = [0, 4.98359521, 5.33717041, -0.13098939];
y2 = [0, 0, 5.25319562, 5.34702277];
x3 = [0, 3.6322271, 7.48338741, 3.73607429];
y3 = [0, -3.52830101, 0, 3.90894884];
x4 = [0, 5.00251036, 5.26071622, -0.17547143];
y4 = [0, 0, 5.23237409, 5.32415307];
x5 = [0, 5.13105963, 5.14021682, -0.16240204, 2.4662067];
y5 = [0, 0, 5.53919407, 5.24762352, 2.7646217];
x6 = [0, 5.22380811, 5.16782235, -0.18729444, 2.49729232];
y6 = [0, 0, 5.58465756, 5.15458453, 2.75053232];

x_real1 = [0, 5, 5, 0];
y_real1 = [0, 0, 5, 5];
x_real2 = [0, 5, 5, 0];
y_real2 = [0, 0, 5, 5];
x_real3 = [0, 3.54, 7.07, 3.54];
y_real3 = [0, -3.54, 0, 3.54];
x_real4 = [0, 5, 5, 0];
y_real4 = [0, 0, 5, 5];
x_real5 = [0, 5, 5, 0, 2.5];
y_real5 = [0, 0, 5, 5, 2.5];
x_real6 = [0, 5, 5, 0, 2.5];
y_real6 = [0, 0, 5, 5, 2.5];


%x1 = [0, 4.66588891, 4.87493685, -0.12007826];
%y1 = [0, 0, 4.79613442, 4.90847974];
%x2 = [0, 4.59531664, 4.93376301, -0.11937696];
%y2 = [0, 0, 4.85156591, 4.94079738];
%x3 = [0, 3.35150777, 6.913225, 3.45311653];
%y3 = [0, -3.25309991, 0, 3.61370987];
%x4 = [0, 4.61206485, 4.85601493, -0.16622938];
%y4 = [0, 0, 4.8308716, 4.91787144];
%x5 = [0, 4.71193479, 4.72081654, -0.14890702, 2.27025352];
%y5 = [0, 0, 5.10507717, 4.82548269, 2.53889093];
%x6 = [0, 4.80289365, 4.75267032, -0.17018244, 2.30109963];
%y6 = [0, 0, 5.14752373, 4.73791577, 2.52600806];

%x_real1 = [0, 4.82, 4.82, 0];
%y_real1 = [0, 0, 4.82, 4.82];
%x_real2 = [0, 4.82, 4.82, 0];
%y_real2 = [0, 0, 4.82, 4.82];
%x_real3 = [0, 3.41, 6.82, 3.41];
%y_real3 = [0, -3.41, 0, 3.41];
%x_real4 = [0, 4.82, 4.82, 0];
%y_real4 = [0, 0, 4.82, 4.82];
%x_real5 = [0, 4.82, 4.82, 0, 2.41];
%y_real5 = [0, 0, 4.82, 4.82, 2.41];
%x_real6 = [0, 4.82, 4.82, 0, 2.41];
%y_real6 = [0, 0, 4.82, 4.82, 2.41];

error1 = sqrt((x1 - x_real1).^2 + (y1 - y_real1).^2)
error2 = sqrt((x2 - x_real2).^2 + (y2 - y_real2).^2)
error3 = sqrt((x3 - x_real3).^2 + (y3 - y_real3).^2)
error4 = sqrt((x4 - x_real4).^2 + (y4 - y_real4).^2)
error5 = sqrt((x5 - x_real5).^2 + (y5 - y_real5).^2)
error6 = sqrt((x6 - x_real6).^2 + (y6 - y_real6).^2)
avg_err_1 = mean(error1(2:4))
avg_err_2 = mean(error2(2:4))
avg_err_3 = mean(error3(2:4))
avg_err_4 = mean(error4(2:4))
avg_err_5 = mean(error5(2:5))
avg_err_6 = mean(error6(2:5))

figure(1);
hold on;
plot(x1, y1, 'rx');
plot(x_real1, y_real1, 'o', 'color', gr);
axis([-1, 6, 0, 6], 'equal');
title("UWB node position reconstruction");
xlabel("x axis [m]");
ylabel("y axis [m]");
legend(["UWB node position reconstructed"; "real UWB node position"], 'location', 'southoutside');
for i = [1:length(x_real1)]
  text(x_real1(i) + 0.2, y_real1(i) + 0.2, num2str(i - 1));
end
grid on;
print("-depsc", "maps/map1.eps")
print("-depslatex", "../../../../ma-thesis/pics/eps/map1.eps")

figure(2);
hold on;
plot(x2, y2, 'rx');
plot(x_real2, y_real2, 'o', 'color', gr);
axis([-1, 6, 0, 6], 'equal');
title("UWB node position reconstruction");
xlabel("x axis [m]");
ylabel("y axis [m]");
legend(["UWB node position reconstructed"; "real UWB node position"], 'location', 'southoutside');
for i = [1:length(x_real2)]
  text(x_real2(i) + 0.2, y_real2(i) + 0.2, num2str(i - 1));
end
grid on;
print("-depsc", "maps/map2.eps")
print("-depslatex", "../../../../ma-thesis/pics/eps/map2.eps")

figure(3);
hold on;
plot(x3, y3, 'rx');
plot(x_real3, y_real3, 'o', 'color', gr);
axis([-1, 8, -4.5, 4.5], 'equal');
title("UWB node position reconstruction");
xlabel("x axis [m]");
ylabel("y axis [m]");
legend(["UWB node position reconstructed"; "real UWB node position"], 'location', 'southoutside');
for i = [1:length(x_real3)]
  text(x_real3(i) + 0.2, y_real3(i) + 0.2, num2str(i - 1));
end
grid on;
print("-depsc", "maps/map3.eps")
print("-depslatex", "../../../../ma-thesis/pics/eps/map3.eps")

figure(4);
hold on;
plot(x4, y4, 'rx');
plot(x_real4, y_real4, 'o', 'color', gr);
axis([-1, 6, 0, 6], 'equal');
title("UWB node position reconstruction");
xlabel("x axis [m]");
ylabel("y axis [m]");
legend(["UWB node position reconstructed"; "real UWB node position"], 'location', 'southoutside');
for i = [1:length(x_real4)]
  text(x_real4(i) + 0.2, y_real4(i) + 0.2, num2str(i - 1));
end
grid on;
print("-depsc", "maps/map4.eps")
print("-depslatex", "../../../../ma-thesis/pics/eps/map4.eps")

figure(5);
hold on;
plot(x5, y5, 'rx');
plot(x_real5, y_real5, 'o', 'color', gr);
axis([-1, 6, 0, 6], 'equal');
title("UWB node position reconstruction");
xlabel("x axis [m]");
ylabel("y axis [m]");
legend(["UWB node position reconstructed"; "real UWB node position"], 'location', 'southoutside');
for i = [1:length(x_real5)]
  text(x_real5(i) + 0.2, y_real5(i) + 0.2, num2str(i - 1));
end
grid on;
print("-depsc", "maps/map5.eps")
print("-depslatex", "../../../../ma-thesis/pics/eps/map5.eps")

figure(6);
hold on;
plot(x6, y6, 'rx');
plot(x_real6, y_real6, 'o', 'color', gr);
axis([-1, 6, 0, 6], 'equal');
title("UWB node position reconstruction");
xlabel("x axis [m]");
ylabel("y axis [m]");
legend(["UWB node position reconstructed"; "real UWB node position"], 'location', 'southoutside');
for i = [1:length(x_real6)]
  text(x_real6(i) + 0.2, y_real6(i) + 0.2, num2str(i - 1));
end
grid on;
print("-depsc", "maps/map6.eps")
print("-depslatex", "../../../../ma-thesis/pics/eps/map6.eps")
