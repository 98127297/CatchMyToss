function [New_Vector] = estimate_vector1(V,t,n)

% estimate the track by m points using pesudo inverse
% the track will be parabolic by least square


dt=mean(diff(t));

m=length(t);
t = t - t(round(m/2));  % reduce condition number
dt=dt/4;
tt=t(m) + dt:dt: t(m) + n*dt;

% should be working good but large error because many noise

% T1 = [t ; ones(1,length(t))];
T2 = [t.^2 ; t ; ones(1,length(t))];

% TT1 = [tt ;ones(1,length(tt))];
TT2 = [tt.^2 ; tt ;ones(1,length(tt))];

New_Vector = V*pinv(T2)*TT2;

end         

