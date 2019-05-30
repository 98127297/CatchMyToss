% Sheffield Dong, 98127297
% Catching main
close all
clear all
clc

%Centre position of bounding box
qCentre = deg2rad([90 0 80 -70 90 0]);

%From base to camera, x = -0.2m, y = 0.06m, z = 0.24m, rotation along x axis = -70deg
baseToCamera = trotz(-90,'deg') * transl(-0.2,0.06,0.24) * trotz(-90,'deg') * trotx(-70,'deg');

%Basket centre is 8cm from end effector in Z axis
% basketOffset = 0.08;

%boundary limits
boundaryLimits = [0.205,0.02;-0.3,-0.4068];

ur3RosControl = UR3RosControl(false);
ur3 = UR3Catching(transl(0,0,0));

%Z Plane height & end effector angle
zPlane = -0.15;
endEffectorAngle = trotx(1.7453);


traj = TrajPrediction(ur3,ur3RosControl,zPlane,baseToCamera,boundaryLimits,qCentre,endEffectorAngle);
