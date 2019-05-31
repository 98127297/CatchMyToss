

%% Robotics Assignment 2 Throwing test

clear all

roscontrol = UR3RosControl(true);

ur3 = UR3('UR3', transl(0,0,0), [-1 1 -1 1 -1 1]);
ur3.model.tool = transl(0,0,0.12);

arduino = arduino();
gripper = Gripper(arduino, 'D6');

ur3.PlotAndColourRobot()

%% Execute pick-up and throw

angles = [85];

for i = 1:size(angles)
    % Go to somewhere above the ball
    goal_1 = transl(0.3, 0, 0.2);
    orien_1 = troty(180,'deg')*trotz(-90,'deg');
    q_matrix_1 = ur3.GoToPose(goal_1, orien_1,false);
    goal_q_1 = q_matrix_1(end,:);

    roscontrol.Ur3_Move(goal_q_1,5);

    % Open gripper
    gripper.Open();

    % Go to pick the ball
    goal_2 = transl(0.3, 0, 0.025);
    orien_2 = troty(180,'deg')*trotz(-90,'deg');
    q_matrix_2 = ur3.GoToPose(goal_2, orien_2,false);
    goal_q_2 = q_matrix_2(end,:);

    roscontrol.Ur3_Move(goal_q_2,5);
    pause(1)
    % Close gripper
    gripper.Hold();
    pause(1)
    % Go to start throwing pos
    %[90 -110 110 -80 -90 0]
    offset_angle = angles(1,i);
    throw_q = deg2rad([offset_angle -110 110 -80 -90 0]);
    roscontrol.Ur3_Move(throw_q,5);

    pause(1)

    % Call the throw service
    try
        roscontrol.Ur3_Throw(true);
    end

    count = 1;

    while true

        q = roscontrol.GetJointAngle();

        if q(1,4) < -2.68  %2.17
            % release the gripper
            gripper.Release();
            break
        end

        count = count + 1;
        if count > 300
            break
        end
    end

end

















