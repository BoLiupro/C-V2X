function Result=Kalman(mode);
%基于弱GNSS信号场景下的C-V2X 同步定位研究
%IMU+卡尔曼滤波
clc;

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
Q = eye(6);

% Measurement noise (sensor uncertainty)
R = 100*[20 0 0;0 0.1 0;0 0 0.05];

% Initial estimation error covariance
P = eye(6);

% Preallocate for plotting
true_position = true_pos(mode);
measured_position = zeros(3, length(t));
estimated_position = zeros(3, length(t));
velocity = zeros(1, length(t));

for i = 1:length(t)
    %误差模块
    %载波测量误差、测边雷达误差（m）、DEM误差（m）
    noise=NoiseSet(mode);
       
    % Simulate the GNSS measurement with noise
    z = SPP(mode,true_position(:,i),x(1:3),noise);
    measured_position(:,i) = z;
    
    if i==1
        estimated_position(:,i) = z;
        velocity(i) = x(4);
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
    velocity(i) = x(4);
end
Result=[measured_position;estimated_position];
end
