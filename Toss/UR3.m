classdef UR3 < handle
    properties (Access=public)
        %> Robot model
        model;
        robotName;
        baseTransform;
    end
    
    properties (Access=private)
        %> workspace
        workspace;   
        ideal_q = deg2rad([90 -90 90 -90 -90 0]);
        t = 0:0.05:8;
        %use for alligning the arm with the item
        rotation_offset = deg2rad(20);  
    end
    
    properties (Constant)
        reachabilityTolereance = 0.02;
    end
    
    % Class for UR3 robot simulation
    methods
        function self = UR3(nameInp, baseTransform, workspace)
                        
            self.robotName = nameInp;
            self.baseTransform = baseTransform;
            self.workspace = workspace;
            
            self.GetUR3Robot();
            %self.PlotAndColourRobot();%robot,workspace);
            %self.model.plot([0 0 0 0 0 0])
            %drawnow    
            % camzoom(2)
            % campos([6.9744    3.5061    1.8165]);

%             camzoom(4)
%             view([122,14]);
%             camzoom(8)
%             teach(self.model);
        end

        % GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetUR3Robot(self)
            pause(0.001);

            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0); % was 'offset',pi/2
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0); % was 'offset',pi/2
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L6 = Link('d',0.0823,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',self.robotName);
            
            self.model.base = self.baseTransform;
        end

        % PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
 
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['ur3link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            
            %Display robot
            self.model.plot3d(self.ideal_q,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.model.delay = 0;
            
            %Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        % Demo
        function plotDemo(self)
            q1 = load('ur3_q.mat');
            
            for i = 1:10:size(q1.q,1)
                self.model.animate(q1.q(i,:)) 
                drawnow()
            end
        end
    
        % Go to pose
        function [q_Matrix, final_Pose] =  GoToPose(self,targeted_t,orientation,animate)
            
            ideal = self.ideal_q;
            current_q = self.model.getpos();
            ideal(1,1) = current_q(1,1);
            q_Matrix = zeros(322,6);          

            %rotate the base so that the end-effector is about 5 degree
            %anti-clockwise from the targeted_t
            
            z = targeted_t(3,4) - self.baseTransform(3,4);
            y = targeted_t(2,4) - self.baseTransform(2,4);
            x = targeted_t(1,4) - self.baseTransform(1,4);
            prefered_angle = FindTargetAngle(y,x, deg2rad(5));
            
            q_target = ideal;
            q_target(1,1) = prefered_angle;
            
            %if target is higher than 0.5m, rotate end-effector upwards
            if z > 0.45
                q_target(1,5) = deg2rad(90);
            end
            
            %if target is at y > 0, rotate end-effector 180
            if y > 0
                q_target(1,6) = deg2rad(180);
            end
            
            %move to ideal initial position
            q_initial =  mtraj(@lspb, self.model.getpos(), q_target, self.t);
            
            if animate == true
                for i = 1:2:size(q_initial,1)
                    self.model.animate(q_initial(i,:))
                    drawnow
                end
            else
                q_Matrix(1:size(self.t,2),:) = q_initial;
            end
            
            %determine the best pose for the end-effector to approach the
            %target
            % if orientation is not given, aka = eye(4), calculate the
            % best pose to approach the target
            
            object_desired_t = eye(4);
            best_maniply = 0;
            q0 = q_target;
               
            %at x, y and x, -y, flip angle of rotation
            if x >= 0 && y >= 0
                flip = true;
            elseif x >= 0 && y <= 0
                flip = true;
            else
                flip = false;
            end
            
            if orientation == eye(4)
                if z > 0.45
                    starting_angle = 270;
                else
                    starting_angle = 160;
                end
                
                for i = starting_angle:10:360
                    if flip == true
                        t = transl( targeted_t(1,4), targeted_t(2,4), targeted_t(3,4) ) * troty(-i,'deg');
                    else
                        t = transl( targeted_t(1,4), targeted_t(2,4), targeted_t(3,4) ) * troty(i,'deg');
                    end
                    q1 = self.model.ikcon(object_desired_t, q0);
                    mani = self.model.maniplty(q1,'all');
                    
                    if mani > best_maniply
                        best_maniply = mani;
                        object_desired_t = t;
                    end
                end
            else %custom
                object_desired_t =  transl( targeted_t(1,4), targeted_t(2,4), targeted_t(3,4) ) * orientation;
            end
            
            %for debugging
            %tranimate(eye(4),object_desired_t,'fps',30);
            
            %move robot arm to target
            
            qobj = self.model.ikcon(object_desired_t, q0);
            
            q_pick = mtraj(@lspb, q_target, qobj, self.t);
            
            if animate == true
                for i = 1:2:size(q_pick,1)
                    self.model.animate(q_pick(i,:))
                    drawnow
                end
            else
                q_Matrix(162:end,:) = q_pick;
            end
            
            final_Pose = self.model.fkine(self.model.getpos());
            
        end %Go to pose
        
        function self = GoToIdealHome(self)
            
            q_initial =  mtraj(@lspb, self.model.getpos(), self.ideal_q, self.t);
            
            for i = 1:3:size(q_initial,1)
                self.model.animate(q_initial(i,:))
                drawnow
            end
            
        end %Go To Ideal Home
        
        % Determine if object is within reach
        % this function is similar to GoToPose function, it determine if
        % the algorithm to move the robot arm is able to reach the given
        % target transform
        function reachable = TargetPoseReachability(self, target_t)
            
            ideal = self.ideal_q;
            
            z = target_t(3,4) - self.baseTransform(3,4);
            y = target_t(2,4) - self.baseTransform(2,4);
            x = target_t(1,4) - self.baseTransform(1,4);
            prefered_angle = FindTargetAngle(y,x, deg2rad(5));
            
            q_target = ideal;
            q_target(1,1) = prefered_angle;
            
            %if target is at y > 0, rotate end-effector 180
            if y > 0
                q_target(1,6) = deg2rad(180);
            end
            
            %if target is higher than 0.5m, rotate end-effector upwards
            if z > 0.45
                q_target(1,5) = deg2rad(90);
            end
            
            if x >= 0 && y >= 0
                flip = true;
            elseif x >= 0 && y <= 0
                flip = true;
            else
                flip = false;
            end
            
            object_desired_t = eye(4);
            best_maniply = 0;
            q0 = q_target;
            
            if z > 0.45
                starting_angle = 270;
            else
                starting_angle = 160;
            end
            
            for i = starting_angle:10:360
                if flip == true
                    t = transl( target_t(1,4), target_t(2,4), target_t(3,4) ) * troty(-i,'deg');
                else
                    t = transl( target_t(1,4), target_t(2,4), target_t(3,4) ) * troty(i,'deg');
                end
                q1 = self.model.ikcon(object_desired_t, q0);
                mani = self.model.maniplty(q1,'all');
                
                if mani > best_maniply
                    best_maniply = mani;
                    object_desired_t = t;
                end
            end
            
            %find joint angle
            q_cal = self.model.ikcon(object_desired_t, q0);
            
            endeffectorFinal_t = self.model.fkine(q_cal);
            
            endeffectorPosition = round(endeffectorFinal_t(1:3,4), 2);
            targetPosition = round(target_t(1:3,4), 2);
            
            %compare the target and actual endeffector position with a
            %tolerance
            comparision = targetPosition - endeffectorPosition
            
            if comparision <= self.reachabilityTolereance
                reachable = true;
            else
                reachable = false;
            end
            
        end %Target Pose Reachability
    end
end

function output = FindTargetAngle(y,x,offset)
%PI22PI convert range of -pi -> pi to 0 -> 2pi

%     if x > 0 && y > 0       %first quadrant
%         rad = atan2(y, x);
%     elseif x < 0 && y > 0   %second quadrant
%         rad = atan2(y, x);
%     elseif x < 0 && y < 0   %third quadrant
%         rad = 2*pi + atan2(y, x);
%     elseif x > 0 && y < 0   %forth quadrant
%         rad = 2*pi + atan2(y, x);
%     end

    if x == 0
        if y > 0
            rad = (3/2)*pi;
        else
            rad = pi/2;
        end
    elseif y == 0
        if x > 0
            rad = pi;
        else
            rad = 0;
        end
    elseif x > 0 && y > 0       %first quadrant
        rad = pi + atan2(y, x) + offset;
    elseif x < 0 && y > 0   %second quadrant
        rad = pi + atan2(y, x) + offset;
    elseif x < 0 && y < 0   %third quadrant
        rad = pi + atan2(y, x) + offset;
    elseif x > 0 && y < 0   %forth quadrant
        rad = pi + atan2(y, x) + offset;
    end

    if rad > 2*pi
        output = rad - 2*pi;
    else
        output = rad;
    end
end

% calculate the correct angle/pose of the end-effector to
% approach the target, fix height for calculation = 0.3
% 
% absDistFromBase = sqrt(( targeted_t(1,4)^2 + targeted_t(2,4)^2 ));
% 
% if(targeted_t(3,4) < 0.3)
%     pose_rad = 180 + round(rad2deg(atan2(absDistFromBase, 1.2)), -1)
% else
%     pose_rad = 270 + round(rad2deg(atan2( (targeted_t(3,4)-0.2),absDistFromBase)), -1)
% end
% 
% object_desired_t = transl( targeted_t(1,4), targeted_t(2,4), targeted_t(3,4) ) * troty(180,'deg');


