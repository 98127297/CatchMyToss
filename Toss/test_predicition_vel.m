
%% get data

clear all
clf
path = "/media/nicholas/SSD/Uni/2019/Autumn/Robotics/Assignment 2/bagfile/thurszedreal1.bag";

%% ROSbag test

bag  = rosbag(path);
                                    
ball_pos = select(bag, 'Topic', '/ball_XYZ');
%img = select(bag, 'Topic', '/kinect2/qhd/image_color_rect');

message = readMessages(ball_pos);
%msg = readMessages(img);

%%

clf
hold on
grid on
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
% define transform between origin (ur3 base) and camera
% base_2_ball = camera_2_ball(from topic) x base_2_camera
base_2_camera = trotz(-90,'deg') * transl(-0.2,0.06,0.24) * trotz(-90,'deg') * trotx(-70,'deg');
%base_2_camera = trotz(-90,'deg') * trotx(-90,'deg');

% matrix for traj_prediction (container for data)
tracked_points = zeros(6,3);
time_stamps = zeros(1,6);
t = zeros(1,6);

% for callback
count = 0;
temp = [];

for i = 1:size(message,1)
    
    if i > 5
        % get xyz coordinate from camera
        point3D = transl(message{i}.Point.X/1000, message{i}.Point.Y/1000, message{i}.Point.Z/1000);
        point_T = base_2_camera * point3D;
        point = point_T(1:3,4)';

        temp = [temp; point];
        plot3(point(1,1), point(1,2), point(1,3),'o')

        % fill the matrix
        tracked_points = [tracked_points(2:5,:); point];
        time_stamps = [time_stamps(1,2:5), message{i}.Header.Stamp.Nsec/1000000000];
        count = count + 1;

        % do prediction if size of list == 6
        if count >= 5

            % calculate correct timestamp due to Nsec reset back to 0 when
            % overflow 1s
            t(1,1) = time_stamps(1,1);
            for j = 1:(size(time_stamps,2)-1)
                dt = time_stamps(1,j+1) - time_stamps(1,j);
                if dt < 0
                    dt = 1 + dt;
                end

                t(1,j+1) = dt + t(1,j);
            end

            [x, y, z] = predictTraj(tracked_points,t(1,1:5),-0.15);
            %if x < 3 && y < 3
                x_f = x
                y_f = y
                z_f = z
                
                plot3(x_f, y_f, z_f, 'x');
            %end
            % determine if any of the points in the predicted path is at the
            % same z height is within the restricted region
            %grid on
            %hold on
            %plot3(predicted_path(1,:), predicted_path(2,:), predicted_path(3,:));
            %break
        end
    end
    
end

%plot3(x_f, y_f, z_f, 'x');
%trplot(transl(0,0,0),'color', 'r','length',0.4);
%trplot(base,'color', 'b','length',0.4);
% figure(2)
% f = fit(temp(:,2),temp(:,3),'poly2');
% plot(f,temp(:,2),temp(:,3));

basketOffset = 0.08;
boundaryLimits = [0.205,0.02;-0.3-basketOffset,-0.4068];

top_right = [0.205, -0.3-basketOffset];
top_left = [0.205, -0.4068];
bottom_right = [0.02, -0.3-basketOffset];
bottom_left = [0.02, -0.4068];

rect = [top_right;top_left;bottom_left;bottom_right];
patch(rect(:,1), rect(:,2), -0.15*ones(4,1), 'r');



%% calculate traj from average velocity

function [x_final, y_final, z_final] = predictTraj(V,t,height)

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













