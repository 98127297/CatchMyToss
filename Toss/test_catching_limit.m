

%% test bounding box
clf
qCentre = deg2rad([90 0 80 -70 90 0]);
endEffectorAngle = trotx(1.7453);

workspace = [-0.5 0.5 -0.5 0.5 -0.5 0.5];

ur3 = UR3('UR3',transl(0,0,0),workspace);

ur3.PlotAndColourRobot()

ur3.model.animate(qCentre)

%% new centre
new = transl(0.1124, -0.3469, -0.05) * endEffectorAngle;
q_new_centre = ur3.model.ikcon(new, qCentre);
ur3.model.animate(q_new_centre);

%%

basketOffset = 0;
mid = [0.1124, -0.3469];
%boundaryLimits = [0.205,0.1124;-0.3,-0.45];
boundaryLimits = [0.1124+0.1, 0.1124-0.1;
                 -0.3,-0.45];

top_right_2 = [0.1124+0.1, -0.3];
top_left_2 = [0.1124+0.1, -0.45];
bottom_right_2 = [0.1124-0.1, -0.3];
bottom_left_2 = [0.1124-0.1, -0.45];

bottom_left = transl(0.1124+0.1, -0.3, -0.05) * endEffectorAngle;
top_left = transl(0.1124+0.1, -0.45, -0.05) * endEffectorAngle;
bottom_right = transl(0.1124-0.1, -0.3, -0.05) * endEffectorAngle;
top_right = transl(0.1124-0.1, -0.45, -0.05) * endEffectorAngle;

rect = [top_right_2;top_left_2;bottom_left_2;bottom_right_2];
h = patch(rect(:,1), rect(:,2), -0.05*ones(4,1), 'r');

test = top_left;
q_test = ur3.model.ikcon(test, qCentre);
ur3.model.animate(q_test);
ur3.model.maniplty(q_test)



