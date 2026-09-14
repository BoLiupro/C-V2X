%基于弱GNSS信号场景下的C-V2X 同步定位研究
%IMU+卡尔曼滤波
clear;clc;

mode=1;

% 时间模块
dt = 0.1; % Time step (seconds)
T = 120; % Total simulation time (seconds)
t = 0:dt:T; % Time vector


% 初始状态[X;Y;Z;Vx;Vy;Vz]
x = initialState(mode);

% State transition matrix (A)
A = [1 0 0 dt 0 0;
     0 1 0 0 dt 0;
     0 0 1 0 0 dt;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

% Control input (B) and acceleration (u) - assuming constant acceleration
% for simplicity(IMU)
u = IMU(mode); % Constant acceleration (m/s^2)
B = [0.5*dt^2 0 0;0 0.5*dt^2 0; 0 0 0.5*dt^2;dt 0 0;0 dt 0;0 0 dt];

% Measurement matrix (C) - we only measure position directly
H = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0];
 
% Process noise (model uncertainty)
Q = 1*eye(6);

% Measurement noise (sensor uncertainty)
% R = 100*[20 0 0;0 0.1 0;0 0 0.05];
R = 100*[1.5 0 0;0 0.1 0;0 0 0.5];

% Initial estimation error covariance
P = eye(6);

% Preallocate for plotting
true_position = true_pos(mode);
measured_position = zeros(3, length(t));
estimated_position = zeros(3, length(t));
velocity = zeros(3, length(t));
z=x(1:3,1);

for i = 1:length(t)
    %误差模块
    %载波测量误差、测边雷达误差（m）、DEM误差（m）
    noise=NoiseSet(mode);
    
    %开始计时
    tic
    
    % Simulate the GNSS measurement with noise
    z = SPP(mode,true_position(:,i),z,noise);
    toc
    
    measured_position(:,i) = z;
    
    if i==1
        estimated_position(:,i) = z;
        velocity(:,i) = x(4:6);
        continue;
    end
    
    % Kalman Filter Estimation
    % Prediction
    x_pred = A * x + B * u(:,i-1);
    P_pred = A * P * A' + Q;
    
    % Measurement update
    K = P_pred * H' * inv(H * P_pred * H' + R);
    x = x_pred + K * (z - H * x_pred);
    P = (eye(6) - K * H) * P_pred;
   
    
    % Store estimates for plotting
    estimated_position(:,i) = x(1:3);
    velocity(:,i) = x(4:6);
end

% Plot results
observe_D=1;
figure;
plot(t, true_position(observe_D,:), 'g-', t, measured_position(observe_D,:), 'b.', t, estimated_position(observe_D,:), 'r-');
xlabel('Time (s)');
ylabel('Position (m)');

observe_D=2;
figure;
plot(t, true_position(observe_D,:), 'g-', t, measured_position(observe_D,:), 'b.', t, estimated_position(observe_D,:), 'r-');
xlabel('Time (s)');
ylabel('Position (m)');

observe_D=3;
figure;
plot(t, true_position(observe_D,:), 'g-', t, measured_position(observe_D,:), 'b.', t, estimated_position(observe_D,:), 'r-');
xlabel('Time (s)');
ylabel('Position (m)');


figure;
plot(t, velocity(1,:), 'r');hold on;
plot(t, velocity(2,:), 'b');hold on;
plot(t, velocity(3,:), 'g');hold on;
plot(t, linspace(11.8,9.245,1201), 'r--');hold on;
plot(t, linspace(-11.79,-13.86,1201), 'r--');hold on;
plot(t, linspace(0,0,1201), 'r--');hold on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Estimated Vehicle Velocity');

%误差
figure;
plot(t, (sum((true_position-measured_position).^2)).^(1/2));
% legend('measured error');
hold on;
plot(t, (sum((true_position-estimated_position).^2)).^(1/2), 'r-');
% legend('Estimated error');
title('GDOP');

%
% figure;
% plot(true_position(1,:),true_position(2,:));hold on;
% plot(measured_position(1,:),measured_position(2,:),'b.');hold on;
% plot(estimated_position(1,:),estimated_position(2,:),'r-');

