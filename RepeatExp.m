clc;clear;
mode=2;
true_result=true_pos(mode);
mean_result=zeros(6,1201);
temp_result=zeros(6,1201);
for i=1:50
    fprintf('%d\n',i);
    temp_result=Kalman(mode);
    mean_result=mean_result+temp_result;
end
mean_result=mean_result./50;
measured=mean_result(1:3,:);
estimated=mean_result(4:6,:);
r1=(sum((true_result-measured).^2)).^(1/2);
r2=(sum((true_result-estimated).^2)).^(1/2);
figure;
plot(1:1201, r1 , 'b.');
legend('Measured error');
hold on;
plot(1:1201, r2 , 'r-');
legend('Estimated error');
fprintf('M:%f ',mean(r1));fprintf('E:%f',mean(r2));

