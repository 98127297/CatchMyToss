classdef TrajPrediction < handle
    %TRAJPREDICTION Summary of this class goes here
    %   Detailed explanation goes here
    
    %% Properties
    properties
        
        % assume global origin is 0,0,0 at base of UR3 catcher
        
        % base is 613mm/ 0.613m from the floor
        
        % working area
        % z height at q_mid = -0.15
        % middle = (0.112, -0.3468, -0.156)
        % top right = (0.02, -0.4068, -0.156)
        % bottom left = (0.205, -0.3, -0.156)
        
        % %From base to camera, x = -0.2m, y = 0.06m, z = 0.24m, rotation along x axis = -70deg
        % define transform between origin (ur3 base) to camera
        % baseToCamera = transl(-0.2,0.06,0.24) * trotz(-90,'deg') * trotx(-70,'deg');
        % %Basket centre is 8cm from end effector in Z axis
        % basketOffset = 0.08;
        % boundaryLimits = [0.205-basketOffset,0.02+basketOffset;-0.3-basketOffset,-0.4068+basketOffset];
        
        baseToCamera;
        zPlane;
        endEffectorAngle;
        qCentre;
        
        % matrix for traj_prediction (container for data)
        trackedPoints;
        timeStamps;
        time;
        
        %Inner Bounding box limits (X and Y)
        xMax;
        xMin;
        yMax;
        yMin;
        yCentre = -0.3469;
        
        %Outer Bounding Box
        outerBox;
        outerOffset;
        %UR3 control
        rosControl;
        
        %UR3
        ur3;
        
        % for callback
        count;
        
        % subscriber handle
        ball_XYZ_sub;
    end
    
    %% Methods
    methods
        %% Constructor
%         function self = TrajPrediction(zPlane, ur3roscontrol)
        function self = TrajPrediction(ur3,ur3roscontrol,zPlane,baseToCamera,boundaryLimits,qCentre,endEffectorAngle,outerOffset)
            self.ball_XYZ_sub = rossubscriber('/ball_XYZ', @self.trajectoryPrediction);
            
            % initialize data
            self.zPlane = zPlane;
            self.qCentre = qCentre;
            self.outerOffset = outerOffset;
            self.xMax = boundaryLimits(1,1);
            self.xMin = boundaryLimits(1,2);
            self.yMax = boundaryLimits(2,1);
            self.yMin = boundaryLimits(2,2);
            self.outerBox = [self.xMax + self.outerOffset, self.xMin - self.outerOffset ...
                ; self.yMax, self.yMin - self.outerOffset];
            self.endEffectorAngle = endEffectorAngle;
            self.ur3 = ur3;
            self.baseToCamera = baseToCamera;
            self.count = 0;
            self.trackedPoints = zeros(5,3);
            self.timeStamps = zeros(1,5);
            self.time = zeros(1,6);
            self.rosControl = ur3roscontrol;
            
            %We need to use ikcon a few times to make it faster?
            %Faster = less processing time during actual run
            corner1 = transl(self.xMax,self.yMax,zPlane)*endEffectorAngle;
            corner2 = transl(self.xMin,self.yMin,zPlane)*endEffectorAngle;
            corner3 = transl(self.xMax,self.yMin,zPlane)*endEffectorAngle;
            corner4 = transl(self.xMin,self.yMax,zPlane)*endEffectorAngle;
            self.Ikcon(corner1);
            self.Ikcon(corner2);
            self.Ikcon(corner3);
            self.Ikcon(corner4);
            
            %Initialisation of UR3 arm to starting position (centre of inner bounding box) 
            ur3roscontrol.Ur3_Move(qCentre,5);
            pause(0.15);
%           %Initialisation finished 'Come at me bro' signal
            centrePose = ur3.model.fkine(qCentre);
            up = centrePose * trotx(-10,'deg');
            down = centrePose * trotx(10,'deg');
            qUp = self.Ikcon(up);
            qDown = self.Ikcon(down);
            ur3roscontrol.Ur3_Move(qUp,0.5)
            pause(0.15);
            ur3roscontrol.Ur3_Move(qDown,0.5);
            pause(0.15);
            ur3roscontrol.Ur3_Move(qCentre,0.5);
            
        end
        
        %% Callback
        function self = trajectoryPrediction(self,~,message)
            % get xyz coordinate from camera
            point3D = transl(message.Point.X/1000, message.Point.Y/1000, message.Point.Z/1000);
            baseToBall = self.baseToCamera * point3D;
            
            % fill the matrix
            self.trackedPoints = [self.trackedPoints(2:5,:); baseToBall(1:3,4)'];
            self.timeStamps = [self.timeStamps(1,2:5), message.Header.Stamp.Nsec/1000000000];
            self.count = self.count + 1;
            
            % Do trajectory prediction if size of list == 6
            if self.count >= 6
                
                % calculate correct timestamp due to Nsec reset back to 0 when
                % overflow 1s
                self.time(1,1) = self.timeStamps(1,1);
                for j = 1:(size(self.timeStamps,2)-1)
                    dt = self.timeStamps(1,j+1) - self.timeStamps(1,j);
                    if dt < 0
                        dt = 1 + dt;
                    end
                    
                    self.time(1,j+1) = dt + self.time(1,j);
                end
                
                [x, y] = self.predictTraj(self.trackedPoints,self.time(1,1:5),self.zPlane);
                
                %Offset the y values by a small amount to improve
                %predictions. This is because our prediction model uses
                %projectile motion that doesnt account for air resistance.
                %Predictions tend to overshoot if we don't add this.
                y = y - 0.08;     %Air drag offset
                
                % determine if xyz is within the inner bounding box
%                 If within box, we move towards the prediction
                if self.CheckConstraint(x,y) == true
                    transform = transl(x,y,self.zPlane) * self.endEffectorAngle;
                    % calculate joint angle ikcon
                    goalJoints = self.Ikcon(transform);
                    disp(goalJoints);
                    % call roscontrol to move arm
                    self.rosControl.Ur3_Move(goalJoints,0.45);
                    %Reset arm to centre
                    pause(0.15);
                    self.rosControl.Ur3_Move(self.qCentre,1.5);
                
                % Check if in outer bounding box
                % If in outer bounding box, move towards the limits of the
                % inner bounding box.
                elseif x <= self.outerBox(1,1) && x >= self.outerBox(1,2)
                    if y <= self.outerBox(2,1) && y >= self.outerBox(2,2)
                        [goalX,goalY] = self.OuterBoxTracking(x,y);
                        transform = transl(goalX,goalY,self.zPlane) * self.endEffectorAngle;
                        % calculate joint angle ikcon
                        goalJoints = self.Ikcon(transform);
                        disp(goalJoints);
                        % call roscontrol to move arm
                        self.rosControl.Ur3_Move(goalJoints,0.45);
                        %Reset arm to centre
                        pause(0.15);
                        self.rosControl.Ur3_Move(self.qCentre,1.5);
                    end
                end
%                 %Tracking mode
%                 else
%                 %Track X through trajectory of ball and change Y depending
%                 %on if it's within bounding box
%                 %If tracking flag is set, the robot is receiving valid
%                 %trajectories.
%                 targetX = self.Tracking(self.trackedPoints(end,1));
%                 display(targetX);
%                 transform = transl(targetX,self.yCentre,self.zPlane) * self.endEffectorAngle;
%                 % calculate joint angle ikcon
%                 goalJoints = self.Ikcon(transform);
%                 disp(goalJoints);
%                 % call roscontrol to move arm
%                 self.rosControl.Ur3_Move(goalJoints,0.4);
%                 end
%             elseif self.ur3.model.getpos() ~= self.qCentre
%                 ur3roscontrol.Ur3_Move(qCentre,5);
%                 reset = true;
            end
        end
        
        
        %% Ikcon to find joint angles
        % Considers the centre of our inner bounding box as the previous
        % joint position to ensure correct motion.
        function qGoal =  Ikcon(self,transform)
            qGoal = self.ur3.model.ikcon(transform,self.qCentre);
        end
        
        %% predictTraj equation
        %
        function [x_final, y_final, z_final] = predictTraj(~,V,time,height)
            
            %Grab x,y and z values from data matrix
            x = V(:,1)';
            y = V(:,2)';
            z = V(:,3)';
            
            %Calculate average x,y and z velocities
            x_vel_total = 0;
            y_vel_total = 0;
            z_vel_total = 0;
            for i = 1:size(x,2)-1
                x_vel = (x(1,i+1) - x(1,i)) / (time(1,i+1) - time(1,i));
                y_vel = (y(1,i+1) - y(1,i)) / (time(1,i+1) - time(1,i));
                z_vel = (z(1,i+1) - z(1,i)) / (time(1,i+1) - time(1,i));
                
                x_vel_total = x_vel_total + x_vel;
                y_vel_total = y_vel_total + y_vel;
                z_vel_total = z_vel_total + z_vel;
            end
            
            x_vel_avg = x_vel_total/(size(x,2)-1);
            y_vel_avg = y_vel_total/(size(x,2)-1);
            z_vel_avg = z_vel_total/(size(x,2)-1);
            
            %Calculate time taken to reach height
            %Projectile Motion, assuming gravity = 9.8m/s^2
            c = z(1,end) - height;
            tt = roots([-4.9 z_vel_avg c]);
            
            tt = tt(tt > 0,1);
            
            %Output prediction of XYZ at specified height
            if size(tt,1) > 0
                x_final = x_vel_avg*tt + x(1,end);
                y_final = y_vel_avg*tt + y(1,end);
                z_final = height;
            else
                x_final = 0;
                y_final = 0;
                z_final = 0;
            end
            
        end
        
        %% Check constraint function
        % Check if calculated is within inner bounding box, if not don't move
        function within = CheckConstraint(self, x,y)
            check = false;
            if x <= self.xMax && x >= self.xMin
                if y <= self.yMax && y >= self.yMin
                    within = true;
                    check = true;
                end
            end
            
            if check == false
                within = false;
            end
        end
        
        %% Tracking function
        % Track the x of trajectory and output x,y for UR3 to move to
        function [x,y] = Tracking(self,predictedX)
            if predictedX > self.xMax
                x = self.xMax-0.05
            elseif predictedX < self.xMin
                x = self.xMin+0.05
            else
                x = predictedX;
            end
        end
        
        %% Outer Box tracking
        % If we aren't in the inner bounding box, we hit the edges of our
        % inner bounding box at the max x and y. Detection is based on if
        % trajectory predicts it will land in the outer bounding box
        function [x,y] = OuterBoxTracking(self,predictedX,predictedY)
            %Check x
            if predictedX > self.xMax
                x = self.xMax;
            elseif predictedX < self.xMin
                x = self.xMin;
            else
                x = predictedX;
            end
            %Check y
            if predictedY > self.yMax
                y = self.yMax;
            elseif predictedY < self.yMin
                y = self.yMin;
            else
                y = predictedY;
            end
        end
    end
end

