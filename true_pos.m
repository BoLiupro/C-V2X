function truePos=true_pos(mode);
% mode==1:匀速直线
% mode==2:加速直线
% mode==3:匀速曲线
% 位置记录时间间隔为1s,总记录时长为121s
dt=0.1;
T=120;
t = 0:dt:T;
Pos=zeros(3,length(t));
if mode==1
    %直线隧道，长2000m，宽20m，高10m
    %车速60Km/h(16.67m/s),车走完隧道全程时间为119.9760s,约120s，末位置为2000.4m
    velocity=16.67;
    Pos(1,:)=velocity.*t;
    Pos(2,:)=17;
    Pos(3,:)=1;
end
if mode==2
    %直线隧道，长2000m，宽20m，高10m
    %初始车速为11m/s,末速度为23m/s,加速度为0.1m/s，车走完隧道全程时间为117.6470s,约120s，末位置为2040m
    velocity=11;
    acceleration=0.1;
    for i=1:length(t)
        Pos(1,i)=velocity*((i-1)/10)+0.5*acceleration*((i-1)/10)^2;
        Pos(2,i)=17;
        Pos(3,i)=1;
    end
end
if mode==3
     %车辆路径半径为10005m，总路程约为2001m,每秒圆心角改变0.001666666666667rad
     %车速60Km/h(16.67m/s),车走完隧道全程时间为119.9760s,约120s
     theta_interval=0.001666666666667;
     for i=1:length(t)
        Pos(1,i)=10005*cos(((i-1)/10)*theta_interval+pi/4);
        Pos(2,i)=10005*sin(((i-1)/10)*theta_interval+pi/4);
        Pos(3,i)=10;
    end
end
truePos=Pos;
end
 

