function measureIndex=primaryBSSelection(mode,tol_RSU,X_hat);
%主基站选择
measure_index=0;
if mode==3
    angle=atan(X_hat(2)/X_hat(1))-pi/4;
    measure_index=fix(angle/0.01)+1;%measure_index 为头部的RSU
else
    for t=1:size(tol_RSU,2)
    if X_hat<tol_RSU(1,t)
        measure_index=t-2; %measure_index 为头部的RSU
        break;
    end
end
end
measureIndex=measure_index;
end