%基于弱GNSS信号场景下的C-V2X 同步定位研究
%IMU+卡尔曼滤波
clear;clc;

mode=3;

% 时间模块
dt = 0.1; % Time step (seconds)
T = 120; % Total simulation time (seconds)
t = 0:dt:T; % Time vector

% Preallocate for plotting
true_position = true_pos(mode);
measured_position = zeros(3, length(t));
velocity = zeros(3, length(t));


% 初始状态[X;Y;Z;Vx;Vy;Vz]
z = initialState(mode);
z=z(1:4,1);
for i = 1:length(t)
    %误差模块
    %载波测量误差、测边雷达误差（m）、DEM误差（m）
    noise=NoiseSet(mode);
    
    % Simulate the GNSS measurement with noise
    z = SPP(mode,true_position(:,i),z,noise);
    measured_position(:,i)=z;
    fprintf('finish at %ds, RSU :%d\nmeasurePosition (%f,%f,%f)\n',i/10,i,z(1),z(2),z(3));
end


%误差分析
subplot(1,2,1);
Root_err_pos_x=sqrt((true_position(2,:)-measured_position(2,:)).^2);
plot(1:1201, Root_err_pos_x, '-o');
xlabel('时间 (s)');
ylabel('水平误差绝对值');
title('水平定位误差');
subplot(1,2,2);
Root_err_pos=sqrt(sum((true_position-measured_position).^2,1));
plot(1:1201, Root_err_pos, '-o');hold on;
plot(1:1201,mean(Root_err_pos));
xlabel('时间 (s)');
ylabel('误差绝对值');
title('绝对定位误差');
