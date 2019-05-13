
% create ros publisher and msg 
pub = rospublisher('/ur_driver/joint_speed','trajectory_msgs/JointTrajectory');
msg = rosmessage(pub);
joint_send = rosmessage('trajectory_msgs/JointTrajectoryPoint');

joint_name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", ...
               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

msg.Points = joint_send;
msg.JointNames = joint_name;


%create UR3 model
L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0); % was 'offset',pi/2
L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0); % was 'offset',pi/2
L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
L6 = Link('d',0.0823,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

UR3_1 = SerialLink([L1 L2 L3 L4 L5 L6],'name','UR3');
            
% test joint angles for publishing movement
beginning_q = deg2rad([0 -90 0 -90 0 0]);
end_q = deg2rad([90 -90 90 -90 -90 0]);

%number of steps for the trajectory
steps = 200;

% get trajectory
q_matrix = mtraj(@lspb, beginning_q, end_q, steps);

%calculate velocity at each trajectory point
velocity = zeros(steps,6);
acceleration  = zeros(steps,6);
for i = 2:steps
    velocity(i,:) = q_matrix(i,:) - q_matrix(i-1,:);                          % Evaluate relative joint velocity
    acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
end

tic
for i = 1:steps
    
    msg.Header.Stamp = rostime('now');
        
    msg.Points.Positions = q_matrix(i,:);
    msg.Points.Velocities = [0 0 0.3 0 0 0];
    %msg.Points.Accelerations = acceleration(i,:);    
    msg.Points.TimeFromStart = rosduration(toc);
    
    rospublisher('/ur_driver/joint_speed',msg);
end





