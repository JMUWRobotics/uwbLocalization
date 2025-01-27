close all;
clear all;

M = colorgradient([0,0.5,0; 0.9,0.9,0; 0.975,0.25,0; 1,0,0], [1,1,0.15], 500);
or = [1, 0.6, 0];
gr = [0,0.5,0];

max_err = 3;

x_min = -1;
x_max = 12;
y_min = -4;
y_max = 5;

data = csvread("trajectories/2024-12-18/1_5.csv");
n = length(data);
for i = 1:n
  if (data(i,1) != 0)
    st = i;
    break;
  endif
endfor
numMidPos = 2;
stScan = [n];
enScan = [st];

%2024-12-17:
%stScan = [1005,1785,n];
%enScan = [st,1487,2281];

%2024-12-17-2:
%stScan = [890,1807,n];
%enScan = [st,1325,2272];

%2024-12-18:
stScan = [1108,2229,n];
enScan = [st,1602,2680];

for i=1:numMidPos+1
  printf("Run Time Section %d: %f\n",i,(stScan(i)-enScan(i))/10)
endfor
printf("Full Run Time %f\n",(n-st)/10)

numScanFrames = 0;
for i = 1:numMidPos
  numScanFrames += (enScan(i+1) - stScan(i));
endfor

%ekf_state: 1-11
%ekf_var: 12-22
%uwb_robot_pos: 23-25
%uwb_anchor_pos: 26-33
%odom: 34-36
%hector: 37-39

%truth_pos = [0.5379972271067162, 1.1246135038312388, 1.3059962190302266, -0.9670285524110119, -1.115298895998488, 0.41976442986730755, 1.7608554426565388, 2.6605936288136114, 0.6909008300071113, 1.8171807286929316, -1.7764722462193185]; #2024-10-04
%truth_pos = [0.9612933775468444, 5.244348687920962, 0.9795536981918449, 2.5630841778782996, -1.3580147493403998, 2.9464126755035194, 3.6795168541642895, -0.8523943203633499, 4.1839853667673, -1.231541426834111, -1.5125745508132347]; #2024-10-09
%truth_pos = [0.5485919070193704, 4.993901372830134, 1.1750767781963551, -1.1405920151470836, -1.3193056298943266, 2.7552602459666953, -0.7883952820540333, 2.3553075035555593, 3.8805322549991774, -1.0939936267180637, 3.506605282922135]; #2024-10-10
%truth_pos = [2.7515140957892577, 0.19911237289241318, -1.841073716, 0.524334131958888, 4.304349117652699, 1.826525501, 2.1064062733943754, -1.2588366872347918, -0.4975648404024999, 2.7376125341719444, 0.20755819566353723, -1.8171979002872012, 2.9713341258014054, 3.3379049925885935]; #2024-10-11
%truth_pos = [3.973111752299989, -0.9594147173630165, -2.849387649, 1.5480713267048594, -0.8318303239254748, 1.585311376, -0.5917389290585527, -1.6832409232239474, 5.02342321695, -1.9122025109104168, 4.70671266970557, 1.038050879424432, 1.726081552319833, 1.3808325971566497]; #2024-10-18
%truth_pos = [1.6083076251209745, -1.5449849059081648, -1.804271695, -0.3798385067377501, 2.4358729085912505, -0.22007634575127638, -1.8183897373888818, 3.807612861495501, 3.3822442918908, 3.681120840093124, -1.5886253934775008]; #2024-12-12
%truth_pos = [3.02, 0.32, 0.3889631, 1.990629770886519, -1.4276541898548727, -2.115592654, 1.0094887984269143, 1.5321128719517978, 1.4364287601298051, 0.012774054348750355, 1.9309783196797503, 0.31274290994328013, -2.1625287707225005, 2.4341516409435293, 2.0356466703543674, 3.1363180532654167, -2.4617101085539583]; #2024-12-16
%truth_pos = [0.07308885897732406, 0.755268268743298, -2.793196488, 2.4107360657297265, -1.4094878404689524, 0.5625885917565442, -0.5731944689826577, -1.639969119521375, 2.377857367, 2.612723594239688, 1.723072990058047, -0.3134113619833333, -2.6455925340950004, -0.17967479688202104, 1.6819714313935419, 2.21750756963675, -2.631406587184091]; #2024-12-17
%truth_pos = [1.1026716613769531, -2.54508087158203, -1.562631, 1.9258915384161737, -1.932897109994898, 1.609126559, 2.796688863170627, 1.9934734833213046, 1.5597461500781835, 3.0697119611410004, -2.5134033965199998, -0.8507831037396838, 0.876577251194853, 3.3394167911369994, 0.21354459881699966, -1.1513145959480002, -1.596852196172925]; #2024-12-17-2
%truth_pos = [7.025642611847392, -2.7002245971611956, -0.2571129282330737, 8.358900817506447, -3.327925646499937, 3.090498032, 3.172692210778809, -3.9069828534569218, -1.299596598081667, 1.3097160957994998, -4.322653680633125, 6.640689998591667, -0.5041143839166676, 6.702373272688334, -4.5037317763016675, 1.1435327599863043, 0.8927231077882499];  #2024-12-17-3
truth_pos = [7.723912908074214, -0.6909566310010833, -0.27283334192762265, 7.1129580312135365, 0.66368685311541, 2.759643664, 4.156153693527501, -1.4459552730898222, -1.817633689, 2.0833250686940836, -1.8554501980839584, 7.570240385651875, -1.547453711485626, 6.7463577219258335, 2.1461259210349994, 1.2440530280307507, 2.9026115622635]; #2024-12-18

error = sqrt((data(:,1) - data(:,37)).^2 + (data(:,2) - data(:,38)).^2)';
errorOdom = sqrt((data(:,34) - data(:,37)).^2 + (data(:,35) - data(:,38)).^2)';

figure(1);
hold on;

%plot lines:
plot(data(:,1), data(:,2), 'k');
plot(data(:,23), data(:,24), 'r');
plot(data(:,34), data(:,35), 'b');
plot(data(:,37), data(:,38), 'color', or);
plot([0 0], [0 0], 'color', gr);

%plot anchors:
plot(data(n,4), data(n,5), 'xk');
plot(data(n,6), data(n,7), 'xk');
plot(data(n,8), data(n,9), 'xk');
plot(data(n,10), data(n,11), 'xk');
drawEllipse([data(n,4) data(n,5) data(n,15) data(n,16) 0], 'color', 'k');
drawEllipse([data(n,6) data(n,7) data(n,17) data(n,18) 0], 'color', 'k');
drawEllipse([data(n,8) data(n,9) data(n,19) data(n,20) 0], 'color', 'k');
drawEllipse([data(n,10) data(n,11) data(n,21) data(n,22) 0], 'color', 'k');
plot(truth_pos(4+3*numMidPos), truth_pos(5+3*numMidPos), 'x', 'color', gr);
plot(truth_pos(6+3*numMidPos), truth_pos(7+3*numMidPos), 'x', 'color', gr);
plot(truth_pos(8+3*numMidPos), truth_pos(9+3*numMidPos), 'x', 'color', gr);
plot(truth_pos(10+3*numMidPos), truth_pos(11+3*numMidPos), 'x', 'color', gr);

%plot final pos:
plot(data(n,1), data(n,2), 'ok');
plot(data(n,23), data(n,24), 'or');
plot(data(n,34), data(n,35), 'ob');
plot(data(n,37), data(n,38), 'o', 'color', or);
plot(truth_pos(1+3*numMidPos), truth_pos(2+3*numMidPos), 'o', 'color', gr);

%plot intermediate pos:
for i = 1:numMidPos
  plot(data(stScan(i),1), data(stScan(i),2), 'ok');
  plot(data(stScan(i),23), data(stScan(i),24), 'or');
  plot(data(stScan(i),34), data(stScan(i),35), 'ob');
  plot(data(stScan(i),37), data(stScan(i),38), 'o', 'color', or);
  plot(truth_pos(1+3*(i-1)), truth_pos(2+3*(i-1)), 'o', 'color', gr);
endfor

%calc RMSE:
rmseRobot = zeros(1,numMidPos+2);
tmpRobot = 0;
rmseWheelOdom = zeros(1,numMidPos+2);
tmpWheelOdom = 0;
errorHector = zeros(1,numMidPos+1);
rmseAnchor = zeros(1,numMidPos+1);
for i = 1:numMidPos+1
  rmseRobot(i) = sqrt(sum(error(enScan(i):stScan(i)).^2) / (stScan(i)-enScan(i)));
  tmpRobot += sum(error(enScan(i):stScan(i)).^2);
  rmseWheelOdom(i) = sqrt(sum(errorOdom(enScan(i):stScan(i)).^2) / (stScan(i)-enScan(i)));
  tmpWheelOdom += sum(errorOdom(enScan(i):stScan(i)).^2);
  errorHector(i) = sqrt((data(stScan(i),37) - truth_pos(3*(i-1)+1))^2 + (data(stScan(i),38) - truth_pos(3*(i-1)+2))^2);
  tmp = 0;
  for k = 4+3*numMidPos:2:10+3*numMidPos
    tmp += (data(stScan(i),k) - truth_pos(k))^2 + (data(stScan(i),k+1) - truth_pos(k+1))^2;
  endfor
  rmseAnchor(i) = sqrt(tmp / 4);
endfor
rmseRobot(numMidPos+2) = sqrt(tmpRobot / (n - st - numScanFrames));
rmseWheelOdom(numMidPos+2) = sqrt(tmpWheelOdom / (n - st - numScanFrames));

printf("rmseWheelOdom: ");
disp(rmseWheelOdom);
printf("rmseRobot: ");
disp(rmseRobot);
printf("errorHector: ");
disp(errorHector);
printf("rmseAnchor: ");
disp(rmseAnchor);

%plot rmse
x_scope = x_max - x_min;
y_scope = y_max - y_min;
ratio = y_scope / x_scope;
%text(x_max-0.27*x_scope*ratio, y_min+0.05*y_scope/ratio, ["RMSE Robot:  " num2str(rmseRobot(numMidPos+2), "%.2f")]);
%text(x_max-0.27*x_scope*ratio, y_min+0.10*y_scope/ratio, ["RMSE Anchor: " num2str(rmseAnchor(numMidPos+1), "%.2f")]);
%drawBox([x_max-0.275*x_scope*ratio x_max y_min+0.025*y_scope/ratio y_min+0.125*y_scope/ratio], 'color', 'k');
text(x_max-0.35*x_scope*ratio, y_min+0.05*y_scope/ratio, ["RMSE Robot:  " num2str(rmseRobot(numMidPos+2), "%.2f")]);
text(x_max-0.35*x_scope*ratio, y_min+0.10*y_scope/ratio, ["RMSE Anchor: " num2str(rmseAnchor(numMidPos+1), "%.2f")]);
drawBox([x_max-0.355*x_scope*ratio x_max y_min+0.025*y_scope/ratio y_min+0.125*y_scope/ratio], 'color', 'k');

xlim([x_min x_max]);
ylim([y_min y_max]);
axis("equal");
title("Trajectories of various localization methods");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
grid on;
legend({"EKF", "only UWB", "Wheel Odometry", "Hector SLAM", "Riegl VZ-400"}, 'location', 'northeastoutside');
%print -depsc trajectories/2024-12-18/1_5.eps
%print -depslatex ../../../../ma-thesis/pics/eps/2024_12_18_1_5.eps

figure(2);
hold on;

%plot ekf line:
x = data(:,1)';
y = data(:,2)';
z = zeros(size(x));
surf([x;x],[y;y],[z;z],[error;error], 'edgecolor', 'interp', 'linewidth', 2);

%plot var of ekf:
for i = 1:numMidPos+1
  for k = enScan(i):50:stScan(i)
    drawEllipse([x(k) y(k) 3*data(k,12) 3*data(k,13) 0], 'color', 'k');
  endfor
  drawEllipse([x(stScan(i)) y(stScan(i)) 3*data(stScan(i),12) 3*data(stScan(i),13) 0], 'color', 'k');
endfor

%plot anchors:
plot(data(n,4), data(n,5), 'xk');
plot(data(n,6), data(n,7), 'xk');
plot(data(n,8), data(n,9), 'xk');
plot(data(n,10), data(n,11), 'xk');
drawEllipse([data(n,4) data(n,5) data(n,15) data(n,16) 0], 'color', 'k');
drawEllipse([data(n,6) data(n,7) data(n,17) data(n,18) 0], 'color', 'k');
drawEllipse([data(n,8) data(n,9) data(n,19) data(n,20) 0], 'color', 'k');
drawEllipse([data(n,10) data(n,11) data(n,21) data(n,22) 0], 'color', 'k');
plot(truth_pos(4+3*numMidPos), truth_pos(5+3*numMidPos), 'x', 'color', gr);
plot(truth_pos(6+3*numMidPos), truth_pos(7+3*numMidPos), 'x', 'color', gr);
plot(truth_pos(8+3*numMidPos), truth_pos(9+3*numMidPos), 'x', 'color', gr);
plot(truth_pos(10+3*numMidPos), truth_pos(11+3*numMidPos), 'x', 'color', gr);

%plot rmse
%text(x_max-0.27*x_scope*ratio, y_min+0.05*y_scope/ratio, ["RMSE Robot:  " num2str(rmseRobot(numMidPos+2), "%.2f")]);
%text(x_max-0.27*x_scope*ratio, y_min+0.10*y_scope/ratio, ["RMSE Anchor: " num2str(rmseAnchor(numMidPos+1), "%.2f")]);
%drawBox([x_max-0.275*x_scope*ratio x_max y_min+0.025*y_scope/ratio y_min+0.125*y_scope/ratio], 'color', 'k');
text(x_max-0.38*x_scope*ratio, y_min+0.05*y_scope/ratio, ["RMSE Robot:  " num2str(rmseRobot(numMidPos+2), "%.2f")]);
text(x_max-0.38*x_scope*ratio, y_min+0.10*y_scope/ratio, ["RMSE Anchor: " num2str(rmseAnchor(numMidPos+1), "%.2f")]);
drawBox([x_max-0.385*x_scope*ratio x_max y_min+0.025*y_scope/ratio y_min+0.125*y_scope/ratio], 'color', 'k');

xlim([x_min x_max]);
ylim([y_min y_max]);
axis("equal");
title("Final output of the EKF");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
grid on;
h=colorbar;
colormap(M);
caxis([0 max_err]);
ylabel(h,'UWB EKF-SLAM Error');
print -depsc trajectories/2024-12-18/1_5_error_var.eps
print -depslatex ../../../../ma-thesis/pics/eps/2024_12_18_1_5_error_var.eps