function noise=NoiseSet(mode);
% 误差设置模块
% 载波测量误差、测边雷达误差（m）、DEM误差（m）
noise=zeros(1,3);
bounde_1=[5 0.1 0.05];%mode1&mode2下的
bounde_2=[5 0.1 0.05];%mode3下的
if mode==3
    noise(1)=Gaussian(bounde_2(1));
    noise(2)=Gaussian(bounde_2(2));
    noise(3)=Gaussian(bounde_2(3));
else
    noise(1)=Gaussian(bounde_1(1));
    noise(2)=Gaussian(bounde_1(2));
    noise(3)=Gaussian(bounde_1(3));
end
end
