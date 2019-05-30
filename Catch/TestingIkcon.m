clear all
close all
clc
% addpath('/home/sheffield/Desktop/41013 Robotics/Assignments/Assignment 2/GitRepo/Toss');
set(0,'DefaultFigureWindowStyle','docked');
ur3 = UR3Catching(transl(0,0,0.12));
% ur3 = UR3
%Assuming we want the same end effector angle
% angle = troty(-pi/2)*trotz(-pi/2)*trotx(deg2rad(5));
endEffectorAngle = trotx(1.7453);
%Our Z plane = -0.15
zPlane = -0.15;
%Centre position
% centre = transl(-0.6,0,zPlane)*angle;
qCentre = deg2rad([90 0 80 -70 90 0]);
topRight = transl(0.2,-0.3,zPlane)*endEffectorAngle;
bottomRight = transl(0.2,-0.4068,zPlane)*endEffectorAngle;
topLeft = transl(0.02,-0.3,zPlane)*endEffectorAngle;
bottomLeft = transl(0.02,-0.4068,zPlane)*endEffectorAngle;
%From base to camera, x = -0.2m, y = 0.06m, z = 0.24m, rotation along x axis = -70deg
baseToCamera = transl(-0.2,0.06,0.24) * trotz(-90,'deg') * trotx(-70,'deg');
%Basket centre is 8cm from end effector in Z axis
basketOffset = 0.08;
boundaryLimits = [0.205-basketOffset,0.02+basketOffset;-0.3-basketOffset,-0.4068+basketOffset];
reallyOff = transl(1,1,0);
traj = TrajPrediction(ur3,zPlane,baseToCamera,boundaryLimits,qCentre,endEffectorAngle);

%Testing transl speed (no real diff)
% tic
% trans = transl(0.1,0.2,0.3);
% toc
% tic
% iMat = eye(4);
% iMat(1:3,4) = [0.1,0.2,0.3]';
% toc
% 
% tic
% traj.Ikcon(topRight);
% toc
% tic
% traj.Ikcon(topLeft);
% toc
% tic
% traj.Ikcon(topRight);
% toc
% tic
% traj.Ikcon(topLeft);
% toc

% tic
% traj.Ikcon(reallyOff);
% toc
% tic
% traj.Ikcon(reallyOff);
% toc
% tic
% traj.Ikcon(topRight);
% toc
% tic
% traj.Ikcon(topLeft);
% toc
% tic
% traj.Ikcon(bottomLeft);
% toc
% tic
% traj.Ikcon(bottomRight);
% toc
%%
% profile on
% % tic
% newQ = ur3.model.ikcon(topRight,qCentre);
% % toc
% % tic
% newQ = ur3.model.ikcon(topLeft,qCentre);
% % toc
% % tic
% newQ = ur3.model.ikcon(bottomLeft,qCentre);
% % toc
% % tic
% newQ = ur3.model.ikcon(bottomRight,qCentre);
% % toc
% profile off;
% ur3.model.plot(qCentre);
% ur3.model.teach;
% %New position
% newPos = transl(-0.6,-0.2,zPlane)*angle;
% tic
% q = ur3.model.ikcon(newPos,qCentre);
% toc
% ur3.model.animate(q);
% newPos = transl(-0.6,0.2,zPlane)*angle;
% pause(3)
% tic
% q = ur3.model.ikcon(newPos,qCentre);
% toc
% ur3.model.animate(q);
% pause(3)
% newPos = transl(-0.5,0,zPlane)*angle;
% tic
% q = ur3.model.ikcon(newPos,qCentre);
% toc
% ur3.model.animate(q);


% q = deg2rad([0, 50,-43, 0, 93, 0]);
% q1 = deg2rad[(

% hold on
%%
% % close all
%Transform from camera to base coordinate frame
% transform = trotx(pi/2);
% transform = transform * trotz(pi/2);

% trStart1 = transl([0,0,0]);
% trEnd1 = troty(pi/2);
% tranimate(trStart1,trEnd1,'fps',25);
% trStart2 = trEnd1;
% trEnd2 = trStart2*trotz(-pi/2);
% tranimate(trStart2,trEnd2,'fps',25);
% trStart3 = trEnd2;
% transform = trStart3*trotx(pi/2)*trotz(pi/2);
% tranimate(trStart3,transform,'fps',25);
% tranimate(trStart,trEnd,'fps',25);
% trStart = trEnd;
% trEnd = trStart*trotz(pi/2);
% tranimate(trStart,trEnd,'fps',25);



% test = troty(-pi/2) * trotz(-pi/2);
% trplot(test);



