

%% Robotics Assignment 2 Throwing test

roscontrol = UR3RosControl(true);

ur3 = UR3('UR3', transl(0,0,0), [-1 1 -1 1 -1 1])
ur3.model.tool = transl(0,0,0.12)

arduino = arduino();
gripper = Gripper(arduino, 'D6');

ur3.PlotAndColourRobot()

%% control

% Go to somewhere above the ball
goal_1 = transl(0.3, 0, 0.2);
orien_1 = troty(180,'deg')*trotz(-90,'deg');
%ur3.GoToPose(goal_1, orien_1,true)
q_matrix_1 = ur3.GoToPose(goal_1, orien_1,false);
goal_q_1 = q_matrix_1(end,:);

roscontrol.Ur3_Move(goal_q_1);

% Open gripper
gripper.Open();

% Go to pick the ball
goal_2 = transl(0.3, 0, 0.025);
orien_2 = troty(180,'deg')*trotz(-90,'deg');
%ur3.GoToPose(goal_2, orien_2,true)
q_matrix_2 = ur3.GoToPose(goal_2, orien_2,false);
goal_q_2 = q_matrix_2(end,:);

roscontrol.Ur3_Move(goal_q_2);
pause(1)
% Close gripper
gripper.Hold();
pause(1)
% Go to start throwing pos
throw_q = deg2rad([90 -145 -80 -100 90 0]);
roscontrol.Ur3_Move(throw_q);

% Call the throw service
roscontrol.Ur3_Throw(true)

i = 1;
q_test = {};

while true
    q = roscontrol.GetJointAngle();
    
    if q(1,3) > -1.2   %-1.1
        % release the gripper
        gripper.Open();
        break
    end
    
    
    i = i + 1;
    if i > 300
        break
    end
end

% while true
%     q_test{i} = roscontrol.GetJointAngle();
%     
%     i = i + 1;
%     if i > 150
%         break
%     end
% end


