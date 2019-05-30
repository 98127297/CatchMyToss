function [New_Vector] = estimate_vector3(V,t,n)

% estimate the track by m points using phisycs Neuton laws
% the track will be parabolic in y axis use gravity in earth
x=V(1,:);
y=V(2,:);
z=V(3,:);     
m=length(t);

dt=mean(diff(t));
t = t - t(round(m/2));  % reduce condition number
dt=dt/4;
tt=t(m) + dt:dt: t(m) + n*dt;


T1 = [t ;ones(1,length(t))];
% T2 = [t.^2 ; t ;ones(1,length(t))];

% TT1 = [tt ;ones(1,length(tt))];
TT2 = [tt.^2 ; tt ;ones(1,length(tt))];

% g = 9.8 m/s  ,   y=0.5 g t^2  ,y[cm]     theta=0.1rad between camera to the world
a = 9800*cos(0.1);  
arr_x =  x /T1;
arr_y = (y-0.5*a*t.^2)/T1;
arr_z =  z/T1;

New_Vector = [[0,arr_x]; [0.5*a,arr_y] ; [0,arr_z]]*TT2;

end

% diff_t=diff(t);
% arr_x =  [mean(diff(x,2)./(diff_t(1:m-2).^2)) , mean(diff(x,1)./diff_t(1:m-1)) , x(m)];
% arr_y =  [mean(diff(y,2)./(diff_t(1:m-2).^2)) , mean(diff(y,1)./diff_t(1:m-1)) , y(m)];
% arr_z =  [mean(diff(z,2)./(diff_t(1:m-2).^2)) , mean(diff(z,1)./diff_t(1:m-1)) , z(m)];
