% Sheffield Dong, 98127927, 29/5/19
% 41014 Robotics Assignment 2
classdef UR3Catching < handle
    properties
        %Model for the robot [SerialLink]
        model;
        %Name of robot
        name;
        %Base of the robot (4x4 Homogeneous Transform Matrix)
        base;
        %Tool of the robot
        tool;
    end
        
    methods
        function self = UR3Catching(tool)
            self.name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];
            self.base = transl(0,0,0);
            self.tool = tool;
            GetUR3Robot(self);
        end
        
        function moveUR3(self)
            
        end
        
                %% GetUR3Robot function
        % Creates the SerialLink for the UR3 provided the name and base
        % inputs.
        function GetUR3Robot(self)
            pause(0.001);
            
            %UR3 DH Parameters
            L1 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset', 0);
            L2 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0); 
            L3 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L4 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0); 
            L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L6 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
            
            %Assign our object model to the SerialLink of the UR3
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',self.name);
            self.model.base = self.base;
            self.model.tool = self.tool;
        end
    end
end