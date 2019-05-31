

%% 
clf

ur3 = UR3();

workspace = [-1 1 -1 1 -0.05 1];

ur3.model.base = transl(0,0,0.5);
ur3.toolModelFilename = ['box.ply'];

ur3.PlotAndColourRobot();

qStart_catching = deg2rad([90 0 80 -70 90 0]);

% for i = 1:5:size(qMatrix_catching,1)
%     ur3.model.plot3d(qMatrix_catching(i,:));
% end
qEnd_catching_t = transl(ball_stop_point(1,1), ball_stop_point(1,2), tc(3,4)) * trotx(90,'deg');
qEnd_catching = catcher.model.ikcon(qEnd_catching_t, qStart_catching);

ur3.model.animate(qEnd_catching);