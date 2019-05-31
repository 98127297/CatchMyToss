
%% run callback for prediction
rosinit

boundaryLimits = [0.205,0.02;-0.3,-0.4068];
base_2_camera = trotz(-90,'deg') * transl(-0.2,0.06,0.24) * trotz(-90,'deg') * trotx(-70,'deg');
qCentre = deg2rad([90 0 80 -70 90 0]);
endEffectorAngle = troty(1.7453);
zPlane = -0.15;

workspace = [-0.5 0.5 -0.5 0.5 -0.5 0.5];
ur3 = UR3('UR3',transl(0,0,0),workspace);

traj = TrajPrediction(ur3,zPlane,baseToCamera,boundaryLimits,qCentre,endEffectorAngle);


%% determine catching angle
clf

% starting joint angle
q_mid = deg2rad([90 0 80 -70 90 0]);

workspace = [-0.5 0.5 -0.5 0.5 -0.5 0.5];

ur3 = UR3('UR3',transl(0,0,0),workspace);

ur3.PlotAndColourRobot()

ur3.model.animate(q_mid)


%% test

t_mid = ur3.model.fkine(q_mid)

% working area
% z height at q_mid = -0.15
% middle = (0.112, -0.3468, -0.156)

% top right = (0.02, -0.4068, -0.156)
% bottom left = (0.205, -0.3, -0.156)
t_top_right = t_mid;
t_top_right(1:2,4) = [0.02;-0.400];
q_top_right = ur3.model.ikcon(t_top_right, q_mid);

ur3.model.animate(q_top_right)

%% determine if point is within constraints

function within = CheckConstraint(x,y)
    
    if x <= 0.205 && x >= 0.02
        if y <= -0.3 && y >= -0.4068
            within = true;
        end
    else
        within = false;
    end

end











