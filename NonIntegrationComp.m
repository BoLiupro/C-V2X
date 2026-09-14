%基于弱GNSS信号场景下的C-V2X 同步定位研究
%无数据融合对照
%载波相位测量+加权最小二乘估计
clc;
clear;
% rng(42);
record=zeros(1201,8);
 
%载波模块
n = 4;  % 一个时刻利用4个RSU进行测量
C=3*10^8;
lambda = 0.19029367;  % 波长（假设为L1频段的波长，单位：米）
f1 = 1575.42e6;       % L1频段的中心频率（单位：赫兹）
Phi = zeros(1,n);

%实验模式
mode=2;

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

% 接收机坐标的初始值
X_hat =[0 17 1]'; %分别是X0 Y0 Z0 Cdt

for measure_t=0:1200
    %接收站运动模块
    X_true=truePos(:,measure_t+1);%测试站更新
    fprintf('\ntruePosition(%f,%f,%f)\n',X_true(1),X_true(2),X_true(3));
    
    %主基站选择
    measure_index=primaryBSSelection(mode,tol_RSU,X_true);
    RSU=tol_RSU(:,measure_index:measure_index+3);
    
    %载波相位测量模块
    noise=NoiseSet(mode); %对于移动站和参考站分别由n个RSU进行载波相位测量
    for t = 1:n
        %相位=距离/波长+噪音
        Phi(1,t) = norm(RSU(:, t) - X_true)/lambda + noise(1);%测试站
        % 生成伪距观测值(这里可能还需要考虑整周模糊度)
    end
    
    %记录相位测量值
    record(measure_t+1,:)=[RSU(1,:) Phi];

    %载波相位测量模型模块
    % 用来计算迭代次数的num
    num = 0;
    X_hat(4,1)=0;%清空Cdt

    % 迭代开始
    while 1
        for t=1:n                     
           % 循环计算第t个卫星的站星距离
           Rs(t) = sqrt((RSU(1, t)-X_hat(1, 1))^2 + (RSU(2, t) - X_hat(2, 1))^2 + (RSU(3, t) - X_hat(3, 1))^2);
           % 由于接收机钟差造成的距离误差Cdt取一个值
           % 计算第t个站星距离的泰勒展开式中的偏导数l, m, n, dCdt = 1;
           ax(t) = (X_hat(1, 1)-RSU(1, t))./Rs(t);
           ay(t) = (X_hat(2, 1)-RSU(2, t))./Rs(t);
           az(t) = (X_hat(3, 1)-RSU(3, t))./Rs(t);
           dCdt(t) = 1;
           %计算自由项
           Lvps(t,1) = Phi(t)*lambda-Rs(t)+X_hat(4); %site_0（4)是Cdt
        end
        % 待到1-6个卫星都被计算后 ，将所有的卫星的系数组成误差方程，以(x, y, z, cdt)为未知数进行求解 
        Avps = [ax', ay', az', dCdt'];

        %WLS
        %Design Matrix
        A=[Avps];
        L=[Lvps];
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
    Measure_pos(:,measure_t+1)=result;
    fprintf('finish at %ds, RSU :%d\nmeasurePosition (%f,%f,%f)\n',measure_t,measure_index,X_hat(1),X_hat(2),X_hat(3));
end

%误差分析
subplot(1,2,1);
Root_err_pos_x=sqrt((truePos(2,:)-Measure_pos(2,:)).^2);
plot(1:1201, Root_err_pos_x, '-o');
xlabel('时间 (s)');
ylabel('水平误差绝对值');
title('水平定位误差');
subplot(1,2,2);
Root_err_pos=sqrt(sum((truePos-Measure_pos).^2,1));
plot(1:1201, Root_err_pos, '-o');hold on;
plot(1:1201,mean(Root_err_pos));
xlabel('时间 (s)');
ylabel('误差绝对值');
title('绝对定位误差');








