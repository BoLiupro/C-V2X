function initialState=initialState(mode);
if mode==1
    initialState=[0 17 1 16.67 0 0]';
else if mode==2
    initialState=[0 17 1 11 0 0]';
    else
        initialState=[10005*cos(pi/4) 10005*sin(pi/4) 10 -16.67*cos(pi/4) 16.67*sin(pi/4) 0]';
    end
end
end