function tol_RSU=RSU(mode);
%RSU模块 
% mode==1:匀速直线
% mode==2:加速直线
% mode==3:匀速曲线
if mode==3
    %曲线隧道，长2000m，宽20m，高10m，RSU布置在隧道中心线顶部
    %隧道为一段圆弧，圆心角为0.2rad,隧道中心线半径为10000m
    %车辆路径半径为10005m，总路程约为2001m,每秒圆心角改变0.001666666666667rad
    %RSU排布在隧道中心线，间隔对应圆心角为0.01rad,(-0.01~0.22)
    theta=linspace(-0.01,0.22,24)+pi/4;
    X = cos(theta)*10000;
    Y = sin(theta)*10000;
    Z = linspace(10,10,24);
    % S是X, Y, Z的坐标 合矩阵
    RSU = [X; Y; Z];
else
    %直线隧道，长2000m，宽20m，高10m，RSU布置在隧道中心线顶部
    X = linspace(-100,2200,24);
    Y = linspace(10,10,24);
    Z = linspace(10,10,24);
    % S是X, Y, Z的坐标 合矩阵
    RSU = [X; Y; Z];
end
tol_RSU=RSU;
end