%% Clear All
clear all;

%% Global Variables
% Base Locations
catcherBase = transl(0,0,0);
throwerBase = transl(0,0,0);

% Ball's Location
% ball = transl(0,0.1,0)*troty(pi)
% plot_sphere([0,0.1,0], 0.02, 'r')


% Joint Positions for Throwing
qStart = deg2rad([90,-135,-70,-100,-90,0]);
qEnd = deg2rad([90,-100,-70,-100,-90,0]);

% Joint Position for Catching
qCatch = deg2rad([0,-45,90,-45,90,0]);


workspace = [-2 2 -2 2 -0.05 3];

steps = 50;

%% UR3 Robots

catcher = UR3();
thrower = UR3();


%% Picking the Ball up
% qBall = thrower.ikcon(ball);
% %qBall = thrower.maniplty(..)
% qMatrix = mtraj(@lspb,qStart,qBall,steps);
% 
% for t = 1:steps
%     thrower.plot(qMatrix(t,:));
% end
% 
% for t = steps:-1:1
%     thrower.plot(qMatrix(t,:));
% end


%% Throwing
thrower.model.base = transl(0,0,0.966);
thrower.PlotAndColourRobot();
thrower.model.animate(qStart);

catcher.model.base = transl(0,1.2,0)*trotz(pi);
catcher.PlotAndColourRobot();
catcher.model.animate(qStart);
axis equal;
 
pause();

qMatrix = mtraj(@lspb,qStart,qEnd,steps);
q = qMatrix(steps/2,:);
qT = thrower.model.fkine(q);
X = qT(1,4)
Y = qT(2,4)
Z = qT(3,4)
[y,z] = Projectile(Z,steps)

x = zeros(1,steps+1);
for i = 1:steps+1
    x(i) = X;
end

y = y+Y;
ball = [x' y' z'];
count = 1;

% Plotting the simulation
for t = 1:2*steps
    
    if t < steps % plotting the thrower
        thrower.model.animate(qMatrix(t,:));
    end
    
    if t == steps % plotting the catcher
        newQ = catcher.model.ikcon(transl(ball(steps/2,1),ball(steps/2,2),0.5));
        catcher.model.animate(newQ);
    end
    
    if t >= steps/2 %% plotting the golf ball
        try delete(ball_h);end;
        ball_h = GetandMovePart('ball.ply',transl(ball(count,1),ball(count,2),ball(count,3)));
        drawnow()
        count = count+1;
        
    end 
end


%% Function
function [x,z] = Projectile(zOff,steps)
%Projectile Motion Equations
% Initialise Variables
G = 9.81; %m/s^2
angle = pi/4; %degrees
v = 1; %m/s
vX = v*cos(angle);
vZ = v*sin(angle);


%Ball Projectile
xmax = (vX * (vX + sqrt((vZ.^2 + 2 * G * zOff))) / G)    %% Calculates max x distance
xstep = xmax / steps;                        %% Calculates step, always 100 samples
           
x = 0:xstep:xmax;
z = (x * tan(angle) - G/(2*(v.^2)*(cos(angle)).^2)*x.^2) + zOff;


end
%% Import Part and Update Pose
function part_h = GetandMovePart(part,tr)
    [face,vertex,data] = plyread(part,'tri');

    vertexColours = [data.vertex.red,data.vertex.green,data.vertex.blue]/225;

    hold on;

    xOffset = 0;
    yOffset = 0;
    zOffset = 0;

    part_h = trisurf(face,vertex(:,1)+xOffset,vertex(:,2)+yOffset,vertex(:,3)+zOffset,'FaceVertexCData', vertexColours,'EdgeColor','interp','EdgeLighting','flat');

    partVertexCount = size(vertex,1);

    midPoint = sum(vertex)/partVertexCount;
    pVerts = vertex - repmat(midPoint,partVertexCount,1);

    pose = eye(4);
    pose = pose * tr;

    updatedPoints = [pose * [pVerts,ones(partVertexCount,1)]']';  

    part_h.Vertices = updatedPoints(:,1:3);

    hold on;
end






