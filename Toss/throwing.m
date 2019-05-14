

%% Robotics Assignment 2 Throwing test
clf

roscontrol = UR3RosControl(true);

ur3 = UR3('UR3', transl(0,0,0), [-1 1 -1 1 -1 1])
ur3.model.tool = transl(0,0,0.12)

%ur3.PlotAndColourRobot()

%% control

% Go to somewhere above the ball
goal_1 = transl(0.3, 0, 0.2);

ur3.GoToPose(goal_1, troty(180,'deg'),true)
q_matrix_1 = ur3.GoToPose(goal_1, troty(180,'deg'),false)

goal_q_1 = q_matrix_1(end,:)

roscontrol.Ur3_Move(goal_q_1);

% Go to pick the ball
goal_2 = transl(0.3, 0, 0);

ur3.GoToPose(goal_2, troty(180,'deg'),true)
q_matrix_2 = ur3.GoToPose(goal_2, troty(180,'deg'),false)

goal_q_2 = q_matrix_2(end,:)

roscontrol.Ur3_Move(goal_q_2);

% Go to start throwing pos
throw_q = deg2rad([90 -135 -70 -100 90 0]);

roscontrol.Ur3_Move(throw_q);

roscontrol.Ur3_Throw(true)
