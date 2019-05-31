% Sheffield Dong, 98127297
% Catching main
close all
clear all
clc

%Optimised joint angle to encourage arm to find correct position
qOptimize = deg2rad([90 0 80 -70 90 0]);      
%zPlane = -0.2, end effector = trotx(100,'deg'), Centre point (0.1124, -0.3465)

%From base to camera, x = -0.2m, y = 0.06m, z = 0.24m, rotation along x axis = -70deg
baseToCamera = trotz(-90,'deg') * transl(-0.2,0.06,0.24) * trotz(-90,'deg') * trotx(-70,'deg');

%Boundary limits of inner bounding box
boundaryLimits = [0.2124,0.0124;-0.3,-0.42];

%Initialise connection between UR3, Matlab and ROS
ur3RosControl = UR3RosControl(false);   % false = no throwing
ur3 = UR3Catching(transl(0,0,0));       % Base at origin (0,0,0)

%Z Plane height & end effector angle
zPlane = -0.1;                          % Arm catching height
centrePoint = transl(0.1124,-0.37,zPlane);  % Centre of inner bounding box
endEffectorAngle = trotx(100,'deg');        % Angle we want end effector to be for catching
qCentre = ur3.model.ikcon(centrePoint*endEffectorAngle, qOptimize);
outerOffset = 0.1;                          % Offset to create outer bounding box

% Create TrajPrediction object, will create the callback function and
% predict the trajectory of the ball and try to move the UR3 within the
% limits of the inner bounding box to catch it. Height specified by zPlane.
traj = TrajPrediction(ur3,ur3RosControl,zPlane,baseToCamera,boundaryLimits,qCentre,endEffectorAngle, outerOffset);
