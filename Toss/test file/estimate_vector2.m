function [New_Vector] = estimate_vector2(V,t,n)

% estimate the track by m points using phisycs Neuton laws
% the track will be parabolic only in y axis
x=V(1,:);
y=V(2,:);
z=V(3,:);     

m=length(t);
dt=mean(diff(t));
t = t - t(round(m/2));  % reduce condition number
dt=dt/4;
tt=t(m) + dt:dt: t(m) + n*dt;


T1 = [t ;ones(1,length(t))];
T2 = [t.^2 ; t ;ones(1,length(t))];

% TT1 = [tt ;ones(1,length(tt))];
TT2 = [tt.^2 ; tt ;ones(1,length(tt))];

arr_x =  x/T1;
arr_y =  y/T2;
arr_z =  z/T1;

New_Vector = [[0  , arr_x]; arr_y ; [0  , arr_z]]*TT2;

end