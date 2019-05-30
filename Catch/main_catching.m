% Sheffield Dong, 98127297
% Catching main
close all
clear all
clc

%Centre position of bounding box
% qCentre = deg2rad([90 0 80 -70 90 0]);      %zPlane = -0.2, end effector = trotx(100,'deg')
%Centre point (0.1124, -0.3465)

%From base to camera, x = -0.2m, y = 0.06m, z = 0.24m, rotation along x axis = -70deg
baseToCamera = trotz(-90,'deg') * transl(-0.2,0.06,0.24) * trotz(-90,'deg') * trotx(-70,'deg');

%Basket centre is 8cm from end effector in Z axis
% basketOffset = 0.08;

%boundary limits
boundaryLimits = [0.2124,0.0124;-0.3,-0.42];

ur3RosControl = UR3RosControl(false);
ur3 = UR3Catching(transl(0,0,0));

%Z Plane height & end effector angle
zPlane = -0.1;
centrePoint = transl(0.1124,-0.3465,zPlane);
endEffectorAngle = trotx(100,'deg');
qCentre = ur3.model.ikcon(centrePoint*endEffectorAngle);
outerOffset = 0.2;
traj = TrajPrediction(ur3,ur3RosControl,zPlane,baseToCamera,boundaryLimits,qCentre,endEffectorAngle, outerOffset);
