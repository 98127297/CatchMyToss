

%% Kalman filter 2D
clear all
x = 5.3;
y = 3.6;
initialState = [x;0;y;0];
KF = trackingKF('MotionModel','2D Constant Velocity','State',initialState);

vx = 0.2;
vy = 0.1;
T  = 0.5;
pos = [0:vx*T:2 ; 5:vy*T:6]';

for k = 1:size(pos,1)
    pstates(k,:) = predict(KF,T);
    cstates(k,:) = correct(KF,pos(k,:));
end

plot(pos(:,1),pos(:,2),'k.', pstates(:,1),pstates(:,3),'+', ...
    cstates(:,1),cstates(:,3),'o')
xlabel('x [m]')
ylabel('y [m]')
grid
xt  = [x-2 pos(1,1)+0.1 pos(end,1)+0.1];
yt = [y pos(1,2) pos(end,2)];
text(xt,yt,{'First measurement','First position','Last position'})
legend('Object position', 'Predicted position', 'Corrected position')

%% Kalman filter 3D
clf
load('points_6.mat')

x = points(1,1);
y = points(1,2);
z = points(1,3);
initialState = [x;1;0; y;0;0; z;0;-4.9];
KF = trackingKF('MotionModel','3D Constant Acceleration','State',initialState);

pos = points;
T = 0.05;

for k = 1:size(pos,1)
    pstates(k,:) = predict(KF,T);
    cstates(k,:) = correct(KF,pos(k,:));
end

plot3(pos(:,1),pos(:,2), pos(:,3),'k.', pstates(:,1),pstates(:,2),pstates(:,3),'+', ...
   cstates(:,1),cstates(:,2),cstates(:,3),'o')

xlabel('x [m]')
ylabel('y [m]')
grid
xt  = [x-2 pos(1,1)+0.1 pos(end,1)+0.1];
yt = [y pos(1,2) pos(end,2)];
text(xt,yt,{'First measurement','First position','Last position'})
legend('Object position', 'Predicted position', 'Corrected position')

