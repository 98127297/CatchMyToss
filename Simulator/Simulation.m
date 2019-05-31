%% Clear All 
clear all;
clf

%% Global Variables
% Base Locations
catcherBase = transl(0,0,0);
throwerBase = transl(0,0,0);

% Ball's Location
% ball = transl(0,0.1,0)*troty(pi)
% plot_sphere([0,0.1,0], 0.02, 'r')


% Joint Positions for Throwing
qStart_throwing = deg2rad([90,-110,110,-80,-90,0]);
qEnd_throwing = deg2rad([90,-110,-20,-170,-90,0]);

% Joint Position for Catching

qStart_catching = deg2rad([90 0 80 -70 90 0]);


workspace = [-2 2 -2 2 -0.05 3];

steps = 100;

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


%% Throwing Model
thrower.model.base = transl(0,0,0.966);
thrower.toolModelFilename = ['gripper.ply']
thrower.PlotAndColourRobot();
thrower.model.animate(qStart_throwing);
tr = thrower.model.fkine(qStart_throwing);
axis equal
hold on;

%% Catching Model

catcher.model.base = transl(0,1.2,0.3);
catcher.toolModelFilename = ['box.ply'];
catcher.PlotAndColourRobot();
catcher.model.animate(qStart_catching);
tc = catcher.model.fkine(qStart_catching);


%% Import 
bench_h = GetandMovePart('bench.ply',transl(0,-1.2,0.5)*trotz(-90,'deg'));
pause();


%% Running the simulation

% determine start position of the ball and trajectory of the ball
qMatrix_thrower = mtraj(@lspb,qStart_throwing,qEnd_throwing,steps);
release_ball_step = 70;
q_ball_initial = qMatrix_thrower(release_ball_step,:);
q_ball_inital_T = thrower.model.fkine(q_ball_initial);
X = q_ball_inital_T(1,4);
Y = q_ball_inital_T(2,4);
Z = q_ball_inital_T(3,4);
[y,z] = Projectile(Z,steps);

x = zeros(1,steps+1);
for i = 1:steps+1 
    x(i) = X;
end

y = y+Y;
ball = [x' y' z'];

% determine stop location of the ball
for j = 1:size(ball,1)
    if ball(j,3) < 0.1840
        ball_stop_step = j;
        ball_stop_point = ball(j,:);
        break
    end
end

% calculate qMatrix_catching
qEnd_catching_t = transl(ball_stop_point(1,1), ball_stop_point(1,2)+0.05, tc(3,4)+0.05) * trotx(90,'deg');
qEnd_catching = catcher.model.ikcon(qEnd_catching_t, qStart_catching);
qMatrix_catching = mtraj(@lspb,qStart_catching,qEnd_catching,steps);


count = 1;
enable = 0;
release_ball = false;

% Plotting the simulation
for t = 1:2*steps
    
    if t < steps % plotting the thrower
        thrower.model.animate(qMatrix_thrower(t,:));
        if t == release_ball_step
            release_ball = true;
        end
    end
    
    if t > steps % plotting the catcher
        catcher.model.animate(qMatrix_catching(t-steps,:));
    end
    
    % plotting the golf ball
    if release_ball == true
        
        % plot position of the ball
        try delete(ball_h);end
        ball_h = GetandMovePart('ball.ply',transl(ball(count,1),ball(count,2),ball(count,3)));
        drawnow()
        count = count+1;
        enable = 1; 
        
        % plot previous trajectory
        if mod(count,5) == 0
            ball_p = plot3(ball(count,1),ball(count,2),ball(count,3), 'ro');
            drawnow()
        end
        
        % stop the ball at a certain height
        if count == ball_stop_step
            release_ball = false;
        end
        
    end 

 
end


%% Function
function [x,z] = Projectile(zOff,steps)
%Projectile Motion Equations
% Initialise Variables
G = 9.81; %m/s^2
angle = deg2rad(10); %degrees
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






