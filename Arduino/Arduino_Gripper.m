%Sheffield Dong, 98127297
%Arduino Servo Gripper Script
%https://au.mathworks.com/help/supportpkg/arduinoio/ug/control-servo-motors.html
clear all
close all
clc
%Create arduino object (auto finds arduino board type and port)
a = arduino();
%Create a servo object and specify max/min pulses duration
% s = servo(a,'D6');
gripper = Gripper(a, 'D6')
gripper.Close
% for angle = 0:0.2:1
%     current_pos = readPosition(s)
%     writePosition(s, angle)
% end

%% MG996R, default settings
%readPosition(s,0.1) for widest jaws
%readPosition(s,0.45) for closed jaws
%readPosition(s,0.3) for firmly grasping golf ball
%readPosition(s,0.2) for picking up golf ball (few mm between grippers to
%golf ball
%MinPulseDuration: 5.44e-04 (seconds)
%MaxPulseDuration: 2.40e-03 (seconds)

function OpenGrip(servo)
    writePosition(servo,0.1);
end

function CloseGrip(servo)
    writePosition(servo,0.45);
end

function HoldBall(servo)
    writePosition(servo,0.3);
end

function ReleaseBall(servo)
    writePosition(servo,0.2);
end

function ReadyGrip(servo)
    writePosition(servo,0.15);
end