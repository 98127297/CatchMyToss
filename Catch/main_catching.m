% Sheffield Dong, 98127297
% Catching main
close all
clear all
clc
addpath('/home/sheffield/Desktop/41013 Robotics/Assignments/Assignment 2/GitRepo/Toss');
%Centre position of bounding box
qCentre = deg2rad([90 0 80 -70 90 0]);
%From base to camera, x = -0.2m, y = 0.06m, z = 0.24m, rotation along x axis = -70deg
baseToCamera = trotz(-90,'deg') * transl(-0.2,0.06,0.24) * trotz(-90,'deg') * trotx(-70,'deg');
%Basket centre is 8cm from end effector in Z axis
basketOffset = 0.08;
boundaryLimits = [0.205-basketOffset,0.02+basketOffset;-0.3-basketOffset,-0.4068+basketOffset];
ur3RosControl = 1%UR3RosControl(false);


traj = TrajPrediction(ur3,ur3RosControl,zPlane,baseToCamera,boundaryLimits,qCentre,endEffectorAngle);
