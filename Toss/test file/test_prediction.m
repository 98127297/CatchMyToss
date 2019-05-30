
%% get data

clear all
clf
path = "/media/nicholas/SSD/Uni/2019/Autumn/Robotics/Assignment 2/bagfile/wedgoodzed6.bag";
load('points_6.mat')

%% ROSbag test

bag  = rosbag(path);
                                    
ball_pos = select(bag, 'Topic', '/ball_XYZ');
%img = select(bag, 'Topic', '/kinect2/qhd/image_color_rect');

message = readMessages(ball_pos);
%msg = readMessages(img);


%%
clf
% define transform between origin (ur3 base) and camera
% base_2_ball = camera_2_ball(from topic) x base_2_camera
base_2_camera = transl(0,0,0) * trotz(-90,'deg') * trotx(-90,'deg');

% matrix for traj_prediction (container for data)
tracked_points = zeros(5,3);
time_stamps = zeros(5,1);
t = zeros(1,5);

% for callback
count = 0;

for i = 1:size(message,1)
    
    if i > 20
        % get xyz coordinate from camera
        point3D = transl(message{i}.Point.X/1000, message{i}.Point.Y/1000, message{i}.Point.Z/1000);
        point_T = base_2_camera * point3D;
        point = point_T(1:3,4)';

        % fill the matrix
        tracked_points = [tracked_points(2:5,:); point];
        time_stamps = [time_stamps(2:5,:); message{i}.Header.Stamp.Nsec/1000000000];
        count = count + 1;
        
        % do prediction if size of list == 5
        if count >= 5
            
            %calculate correct timestamp
            for j = 1:(size(time_stamps,1)-1)
                dt = time_stamps(j,1) - time_stamps(j+1,1);
                if dt < 0
                    dt = 1 + dt;
                end
                
                t(1,j+1) = dt + t(1,j);
                
            end
            t = flip(t);
            predicted_path = estimate_vector2test(tracked_points,t,20);

            % determine if any of the points in the predicted path is at the
            % same z height is within the restricted region
            grid on
            hold on
            plot3(predicted_path(1,:), predicted_path(2,:), predicted_path(3,:));
        end
    end
    
end

for i = 1:size(points,1)
    plot3(points(i,1), points(i,2), points(i,3),'o')
end

%% 
function [New_Vector] = estimate_vector2test(V,t,n)

% estimate the track by m points using phisycs Neuton laws
% the track will be parabolic only in y axis
x=flip(V(:,1)');
y=flip(V(:,2)');
z=flip(V(:,3)');     

m=length(t);
dt=mean(diff(t));
%t = t - t(round(m/2));  % reduce condition number
dt=dt/4;
tt=t(m) + dt:dt: t(m) + n*dt;


T1 = [t ;ones(1,length(t))];
% T2 = [t.^2 ; t ;ones(1,length(t))];

% TT1 = [tt ;ones(1,length(tt))];
TT2 = [tt.^2 ; tt ;ones(1,length(tt))];

% g = 9.8 m/s  ,   y=0.5 g t^2  ,y[cm]     theta=0.1rad between camera to the world
a = 9800*cos(0.1);  
arr_x =  x /T1;
arr_y = (y-0.5*a*t.^2)/T1;
arr_z =  z/T1;

New_Vector = [[0,arr_x]; [0.5*a,arr_y] ; [0,arr_z]]*TT2;

end