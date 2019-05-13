

JOINT_NAMES = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
           

% action lib library for ur3 arm 
[client, goalMsg] = rosactionclient('/follow_joint_trajectory');

% connect to the server
waitForServer(client);

joint_state_sub = rossubscriber('/joint_states');

joint_trajectory_start = rosmessage('trajectory_msgs/JointTrajectoryPoint');
joint_trajectory_goal = rosmessage('trajectory_msgs/JointTrajectoryPoint');

% get current joint state
current_q = receive(joint_state_sub, 1)
current_q_pos = current_q.Position'
current_q_vel = current_q.Velocity'

joint_trajectory_start.Positions = current_q_pos;
joint_trajectory_start.Velocities = current_q_vel;
joint_trajectory_start.TimeFromStart = rosduration(0.0);

joint_trajectory_goal.Positions = zeros(1,6);
joint_trajectory_goal.Velocities = zeros(1,6);
%joint_trajectory_points.Accelerations = [0.75 0.75 0.75 0.75 0.75 0.75];
joint_trajectory_goal.TimeFromStart = rosduration(5.0);

goalMsg.Trajectory.JointNames = JOINT_NAMES;
goalMsg.Trajectory.Points = [joint_trajectory_start, joint_trajectory_goal];


[resultMsg,resultState] = sendGoalAndWait(client,goalMsg,10);
 
resultMsg
resultState

