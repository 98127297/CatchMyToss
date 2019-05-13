

%% Robotics Assignment 2 Throwing

% open gripper
gripper = Gripper(Arduino, 'D6');
gripper.Ready();

% move to somewhere slightly higher than the ball pos

% move to ball pos (to ensure the end-effector doesn't hit the ball)

% close gripper

% move to starting position ( get a selection of starting position and
% randomize the selection)

% call throw ball service

% listen to topic joint_states, when reaches the desired joint angle,
% release the gripper to throw the ball