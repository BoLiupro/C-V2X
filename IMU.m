function acceleration=IMU(mode);
acceleration=zeros(3,1200);
if mode==1
    acceleration=zeros(3,1200);
else if mode==2
        acceleration(1,:)=0.1;
    else 
        acceleration=zeros(3,1201);
        omega=0.001666666666667;
        r=10005;
        a0=omega^2*r;
         for i=1:1201
            acceleration(1,i)=a0*cos(((i-1)/10)*omega+3*pi/4);
            acceleration(2,i)=a0*sin(((i-1)/10)*omega+3*pi/4);
            acceleration(3,i)=0;
        end
    end
end
end