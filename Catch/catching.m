%% Sheffield Dong, 98127297, 25/4/19
% Assignment 2 Catching
clear all
close all
clc
addpath('/home/sheffield/Desktop/41013 Robotics/Assignments/Assignment 2/GitRepo/Toss');
rosshutdown
%% Initialise UR3 ROS Control
% roscontrol = UR3RosControl(false);
% ur3 = UR3('UR3', transl(0,0,0), [-1 1 -1 1 -1 1]);
% ur3.model.tool = transl(0,0,0.12);        %Consider our box attached to end effector
ur3 = UR3Catching(transl(0,0,0.12));
% ur3.PlotAndColourRobot();
qCentre = deg2rad([90 0 80 -70 90 0]);
ur3.model.plot(qCentre);
ur3.model.teach;
% q2 = deg2rad([-36,50,-50,0,57,0]);
% transform = ur3.model.fkine(q2);
% tic
% newQ = ur3.model.ikcon(transform,q);
% newQ = ur3.model.ikine(transform,q)
% toc

% ur3.model.animate(newQ);
% hold on;
% %% Gather data from ROS topics (Make sure it's valid)
% try
%     rosinit;
% catch
%     display('Rosinit already called, continuing');
% end

% %Subscribe to ball position topic (/ball_XYZ)
% ballSub = rossubscriber('/ball_XYZ', @ballCallBack);
% 
% ballData = receive(ballSub, 10);

%% Convert camera coordinate frame to robot coordinate frame
%https://robotacademy.net.au/lesson/base-and-tool-transforms/
% The camera coordinate frame is considered the 'world coordinate frame'
% Camera coordinate frame will have Z pointing outwards from the camera
% with x pointing towards the right and y pointing downwards.

%Camera to base
% cameraToBase = transl(0,0,0) * trotx(pi);
%We offset our UR3 base by the amount from the camera thats at origin
%NOTE: We can only do this because the position of the camera relative to
%the base of the UR3 is known (we will set it up the same way everytime)
% ur3.model.base = cameraToBase;

% By doing this we don't have to do anymore calculations, the UR3 position
% is accounted for becuase we know how to get from the base to the end
% effector.

%% Predict trajectory


%% Find intersection
%Create square given centre point and size
centrePoint = [-0.5,-0.2,-0.12]
squareSize = 0.25;
square = zeros(4,3);
square(1,:) = [centrePoint(1,1)-squareSize,centrePoint(1,2)-squareSize,centrePoint(1,3)];       %bottom left corner of square
square(2,:) = [centrePoint(1,1)-squareSize,centrePoint(1,2)+squareSize,centrePoint(1,3)]; % Top left corner
square(3,:) = [centrePoint(1,1)+squareSize,centrePoint(1,2)+squareSize,centrePoint(1,3)]; % Top right corner
square(4,:) = [centrePoint(1,1)+squareSize,centrePoint(1,2)-squareSize,centrePoint(1,3)]; % Bottom right corner
x = square(:,1);
y = square(:,2);
z = square(:,3);
patch(x,y,z,'r');       %Bounding box representation
zPlane = [0,0,1];          %Normal plane
zPoint = centrePoint(1,3);
planePoint = transl(0,0,-0.12);
planePoint = planePoint(1:3,4)';
testPoint1 = [-0.5,0,1];
testPoint2 = [-0.5,0,-1];
% [x y] = meshgrid(-0.25:0.1:0.25); % visualise plane
% x = x -0.5; %Move plane centre to centre of UR3's bounding box
% z = zPoint*ones(size(x, 1));  %Generate Z data
% surf(x, y, z) % Plot the surface

% [intersectionPoint,check] = LinePlaneIntersection(zPlane, planePoint, 

% predictedJoints = 
%% Move UR3 to predicted point (within bounding box)
qCentre = deg2rad([0, 50,-43, 0, 93, 0]);
endEffectorAngle = troty(-pi/2)*trotz(-pi/2)*trotx(deg2rad(5));    %Our end effector angle
centrePose = transl(-0.6,0,zPlane)*angle;       %Centre pose of bounding box

goalPose = newXYZ * endEffectorAngle;
qGoal = ur3.model.ikcon(newXYZ);
roscontrol.Ur3_Move(qGoal);

% if (abs(predictedPoint(1)) < maxX && abs(predictedPoint(2)) < maxY
%     catchPosition = predictedPosition;
% else
%     catchPosition = currentPosition;
% end
% 
% catchOrientation = trotx(pi);
% qMatrix = ur3.GoToPose(catchPosition, catchOrientation,false);
% roscontrol.Ur3_Move(qMatrix);