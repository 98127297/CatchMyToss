classdef TrajPrediction < handle
    %TRAJPREDICTION Summary of this class goes here
    %   Detailed explanation goes here
    
    %% Properties
    properties
        
        % assume global origin is 0,0,0
        
        % base is 613mm/ 0.613m from the floor
        
        % working area
        % z height at q_mid = -0.15
        % middle = (0.112, -0.3468, -0.156)
        % top right = (0.02, -0.4068, -0.156)
        % bottom left = (0.205, -0.3, -0.156)
        
        % define transform between origin (ur3 base) and camera
        % base_2_ball = camera_2_ball(from topic) x base_2_camera
        base_2_camera = transl(0,0,0.1) * trotz(-90,'deg') * trotx(-90,'deg');
        
        % matrix for traj_prediction (container for data)
        tracked_points;
        time_stamps;
        t;
        
        %UR3 control
        roscontrol;
        
        % for callback
        count;
        
        % subscriber handle
        ball_XYZ_sub;
    end
    
    %% Methods
    methods
        %% Constructor
        function self = TrajPrediction()
            self.ball_XYZ_sub = rossubscriber('/ball_XYZ', @self.trajectoryPrediction);
            
            % initialize data
            self.count = 0;
            self.tracked_points = zeros(5,3);
            self.time_stamps = zeros(1,5);
            self.t = zeros(1,6);
            %self.roscontrol = ur3roscontrol;
            
            figure(1)
            hold on
            
        end
        
        %% Callback
        function self = trajectoryPrediction(self,~,message)
            
            % get xyz coordinate from camera
            point3D = transl(message.Point.X/1000, message.Point.Y/1000, message.Point.Z/1000);
            point_T = self.base_2_camera * point3D;
            
            % fill the matrix
            self.tracked_points = [self.tracked_points(2:5,:); point_T(1:3,4)'];
            self.time_stamps = [self.time_stamps(1,2:5), message.Header.Stamp.Nsec/1000000000];
            self.count = self.count + 1;
            
            % do prediction if size of list == 6
            if self.count >= 6
                
                % calculate correct timestamp due to Nsec reset back to 0 when
                % overflow 1s
                self.t(1,1) = self.time_stamps(1,1);
                for j = 1:(size(self.time_stamps,2)-1)
                    dt = self.time_stamps(1,j+1) - self.time_stamps(1,j);
                    if dt < 0
                        dt = 1 + dt;
                    end
                    
                    self.t(1,j+1) = dt + self.t(1,j);
                end
                
                [x, y] = self.predictTraj(self.tracked_points,self.t(1,1:5),0.1);
                
                % determine if xyz is within the fixed box
                if self.CheckConstraint(x,y) == true
                    
                    % calculate joint angle ikcon
                    % call roscontrol
                    
                end
                
            end
        end
        
        %% predictTraj equation
        function [x_final, y_final, z_final] = predictTraj(self,V,t,height)
            
            x = V(:,1)';
            y = V(:,2)';
            z = V(:,3)';
            
            % calculate average x vel
            
            % x velocity total
            x_vel_total = 0;
            y_vel_total = 0;
            z_vel_total = 0;
            for i = 1:size(x,2)-1
                
                x_vel = (x(1,i+1) - x(1,i)) / (t(1,i+1) - t(1,i));
                y_vel = (y(1,i+1) - y(1,i)) / (t(1,i+1) - t(1,i));
                z_vel = (z(1,i+1) - z(1,i)) / (t(1,i+1) - t(1,i));
                
                x_vel_total = x_vel_total + x_vel;
                y_vel_total = y_vel_total + y_vel;
                z_vel_total = z_vel_total + z_vel;
            end
            
            x_vel_avg = x_vel_total/(size(x,2)-1);
            y_vel_avg = y_vel_total/(size(x,2)-1);
            z_vel_avg = z_vel_total/(size(x,2)-1);
            
            % calculate time to reach height
            c = z(1,end) - height;
            tt = roots([-4.9 z_vel_avg c]);
            
            tt = tt(tt > 0,1);
            
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
        function within = CheckConstraint(~, x,y)
            
            if x <= 0.205 && x >= 0.02
                if y <= -0.3 && y >= -0.4068
                    within = true;
                end
            else
                within = false;
            end
            
        end
    end
end

