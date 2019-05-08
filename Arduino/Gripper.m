%Sheffield Dong, 98127297, 7/5/19
%41014 Robotics, Assignment 2
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
            self.servo = servo(arduino, pin);
        end
        function Close(self)
            writePosition(self.servo,0.45);
        end
        function Open(self)
            writePosition(self.servo,0.1);
        end
        function Ready(self)
            writePosition(self.servo,0.15);
        end
        function Hold(self)
            writePosition(self.servo,0.3);
        end
        function Release(self)
            writePosition(self.servo,0.2);
        end
    end
end