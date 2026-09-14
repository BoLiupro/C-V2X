function y = Gaussian(bound)
%得到一个只处在固定区间，且满足局部正态分布的随机数
temp=clock; 
temp=sum(temp(4:6))*sum(temp(2:3)); 
temp=round(temp/10); 
rand('seed',temp); 
r=randn;
if r<-1
    y=-bound;
elseif  r>1
    y=bound;
else
    y=bound*r;
end

