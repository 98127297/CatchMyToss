%Sheffield Dong, 98127297, 7/5/19
%41014 Robotics, Assignment 2

%% Servo Specs: MG996R, default settings
% Note: writePosition uses a value between 0 < theta < 1 for angle.
% theta approaching 1 moves servo CCW, theta approaching 0 moves servo CW
%writePosition(s,0.1) for widest jaws
%writePosition(s,0.2) releasing golf ball 
%writePosition(s,0.3) for firmly grasping golf ball
%writePosition(s,0.45) for closed jaws

%MinPulseDuration: 5.44e-04 (seconds)
%MaxPulseDuration: 2.40e-03 (seconds)

%% Gripper Class
%Class for instantiating the gripper through Arduino
classdef Gripper < handle
    %% Properties
    properties
        % Servo object
        servo;
    end
        
    methods
        %Constructor
        function self = Gripper(arduino, pin)
            %Note: This requires the arduino object to be created first
            self.servo = servo(arduino, pin);
        end
        function Close(self)
            writePosition(self.servo,0.45);
        end
        function Open(self)
            writePosition(self.servo,0);
        end
        function Ready(self)
            writePosition(self.servo,0.15);
        end
        function Hold(self)
            writePosition(self.servo,0.1);
        end
        function Release(self)
            writePosition(self.servo,0.2);
        end
    end
end