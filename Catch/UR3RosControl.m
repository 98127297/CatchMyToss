classdef UR3RosControl
    %UR3ROS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        client
        enable_throwing
        throwing_client
        goalMsg
        joint_trajectory_current
        joint_trajectory_goal
        joint_state_sub
        
        JOINT_NAMES = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};

    end
    
    %% Constructor
    methods
        function self = UR3RosControl(enable_throwing)
            
            self.enable_throwing = enable_throwing;
            
            try 
                rosshutdown
            end
            rosinit
            
            if self.enable_throwing
                self.throwing_client = rossvcclient('/ur3_throw')
            end
            
            % create rosaction lib for moving the robot
            [self.client, self.goalMsg] = rosactionclient('/follow_joint_trajectory');
            
            disp('Waiting for Server')
            waitForServer(self.client);
            disp('Connected to Server')
            
            % create rossubscriber for joint_states
            self.joint_state_sub = rossubscriber('/joint_states');
            
            self.joint_trajectory_current = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            self.joint_trajectory_goal = rosmessage('trajectory_msgs/JointTrajectoryPoint');
 
        end
        
        %% Move_Ur3
        function result = Ur3_Move(self, joint_angles, duration)
            
            % check if joint_angles has 1 row 6 col
            if size(joint_angles, 1) == 1 && size(joint_angles,2) == 6
                current_q = receive(self.joint_state_sub, 1);
                current_q_pos = current_q.Position';
                current_q_vel = current_q.Velocity';

                self.joint_trajectory_current.Positions = current_q_pos;
                self.joint_trajectory_current.Velocities = current_q_vel;
                self.joint_trajectory_current.TimeFromStart = rosduration(0.0);

                self.joint_trajectory_goal.Positions = joint_angles;
                self.joint_trajectory_goal.Velocities = zeros(1,6);
                %joint_trajectory_points.Accelerations = [0.75 0.75 0.75 0.75 0.75 0.75];
                self.joint_trajectory_goal.TimeFromStart = rosduration(duration);

                self.goalMsg.Trajectory.JointNames = self.JOINT_NAMES;
                self.goalMsg.Trajectory.Points = [self.joint_trajectory_current, self.joint_trajectory_goal];


                [resultMsg,result] = sendGoalAndWait(self.client,self.goalMsg,10);
            end
            
        end
        
        %% call ur3_throw service
        function result = Ur3_Throw(self, input)
            
            if self.enable_throwing || input == true || input == false
                %reqMsg = rosmessage(self.throwing_client)
                %reqMsg.Throw = input;
                call(self.throwing_client,'Timeout',0.1);
            else
                result = false;   
            end
        end
        
        %% get joint angle
        function q = GetJointAngle(self)
            joint_state = receive(self.joint_state_sub, 1); 
            q = joint_state.Position';
        end        
    end
end

