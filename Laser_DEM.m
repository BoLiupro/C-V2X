function Laser_DEM=Laser_DEM(mode,X_true,noise_2,noise_3);
%LaserºÍDEMÄ£¿é
y_laser=0;
z_dem=0;
if mode==3
    y_laser=X_true(2)+noise_2;
    z_dem=X_true(3)+noise_3;
else
    Yl=0;
    Yr=20;
    W=Yr-Yl;
    d1=X_true(2)-Yl+noise_2;
    d2=Yr-X_true(2)+noise_2;
    y_laser=d1*W/(d1+d2);
    z_dem=X_true(3)+noise_3;
end
Laser_DEM=[y_laser,z_dem]';
end
