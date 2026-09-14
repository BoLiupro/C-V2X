function z=SPP(mode,X_true,X_hat,noise);
%基于弱GNSS信号场景下的C-V2X 同步定位研究
%载波相位测量+加权最小二乘估计
 
%载波模块
n = 4;  % 一个时刻利用4个RSU进行测量
C=3*10^8;
lambda = 0.19029367;  % 波长（假设为L1频段的波长，单位：米）
f1 = 1575.42e6;       % L1频段的中心频率（单位：赫兹）
Phi = zeros(1,n);

%RSU模块
tol_RSU = RSU(mode);

%运动记录
truePos=true_pos(mode);
Measure_pos=zeros(3,121);

% 给需要用到的参数预先分配内存
Rs = zeros(1, n);% 站星距
ax = zeros(1, n);
ay = zeros(1, n);
az = zeros(1, n);
dCdt = zeros(1, n);
Lvps = zeros(n, 1);
Vvps = zeros(n,1);
Vpr = zeros(n,1);
W = zeros(6);

%主基站选择
measure_index=primaryBSSelection(mode,tol_RSU,X_hat);
rsu=tol_RSU(:,measure_index:measure_index+3);

%载波相位测量模块
 %对于移动站和参考站分别由n个RSU进行载波相位测量
for t = 1:n
%         noise(1,t) = 2*pi*rand();  % 随机生成载波相位噪声
    %相位=距离/波长+噪音
    Phi(1,t) = norm(rsu(:, t) - X_true)/lambda + noise(1);%测试站
    % 生成伪距观测值(这里可能还需要考虑整周模糊度)
end

%载波相位测量模型模块
% 用来计算迭代次数的num
num = 0;
X_hat(4,1)=0;%清空Cdt

%Laser和DEM模块
integration_data=Laser_DEM(mode,X_true,noise(2),noise(3));
y_laser=integration_data(1);
z_dem=integration_data(2);

% 迭代开始
while 1
    for t=1:n                     
       % 循环计算第t个卫星的站星距离
       Rs(t) = sqrt((rsu(1, t)-X_hat(1, 1))^2 + (rsu(2, t) - X_hat(2, 1))^2 + (rsu(3, t) - X_hat(3, 1))^2);
       % 由于接收机钟差造成的距离误差Cdt取一个值
       % 计算第t个站星距离的泰勒展开式中的偏导数l, m, n, dCdt = 1;
       ax(t) = (X_hat(1, 1)-rsu(1, t))./Rs(t);
       ay(t) = (X_hat(2, 1)-rsu(2, t))./Rs(t);
       az(t) = (X_hat(3, 1)-rsu(3, t))./Rs(t);
       dCdt(t) = 1;
       %计算自由项
       Lvps(t,1) = Phi(t)*lambda-Rs(t)+X_hat(4); %site_0（4)是Cdt
    end
    % 待到1-6个卫星都被计算后 ，将所有的卫星的系数组成误差方程，以(x, y, z, cdt)为未知数进行求解 
    Avps = [ax', ay', az', dCdt'];

    % integration
    Lpr=[y_laser-X_hat(2),z_dem-X_hat(3)]';
    Apr=[0 1 0 0;0 0 1 0];

    %WLS
    %Design Matrix
    A=[Avps;Apr];
    L=[Lvps;Lpr];
    %第一次使用普通最小二乘回归,后续使用加权最小二乘WLS
    if num==0
        X_update=pinv(A'*A)*(A'*L);
    else    
        W = diag(1./V.^2);
        X_update=pinv(A'*W*A)*(A'*W*L);
    end
    V=A*X_update-L;
    X_hat = X_hat + X_update;
    num = num + 1;
    % 如果deltaX小于0.0001，则说明上述所计算的接收机坐标基本接近实际值，则结束迭代，反之用本次计算出的接收机坐标重新循环迭代
    if abs(X_update(1, 1)) < 0.05 && abs(X_update(2, 1)) < 0.02 & abs(X_update(3, 1)) < 0.02
        result=X_hat(1:3);
        break; % 求得满足条件的XYZ，结束迭代
    end
end
z=result;
end








