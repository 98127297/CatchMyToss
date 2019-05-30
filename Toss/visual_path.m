%Test file

%% Path to bag file
clear all
clf
path = "/media/nicholas/SSD/Uni/2019/Autumn/Robotics/Assignment 2/bagfile/thurszedreal1.bag";

%% ROSbag test

bag  = rosbag(path);
                                    
ball_pos = select(bag, 'Topic', '/ball_XYZ');
%img = select(bag, 'Topic', '/kinect2/qhd/image_color_rect');

msgs = readMessages(ball_pos);
%msg = readMessages(img);

%% plot 3d points
clf
figure(1)
hold on
grid on
%axis = [0 5 -1 1 -1 1];
axis equal

% camera Z = global x
% camera X = global -y
% camera Y = global -z


x = zeros(1,size(msgs,1));
y = zeros(1,size(msgs,1));
z = zeros(1,size(msgs,1));

raw_data = zeros(size(msgs,1), 4);

% convert msg data to points 
for i = 1:size(msgs,1)
    x(1,i) = msgs{i}.Point.X/1000;
    y(1,i) = msgs{i}.Point.Y/1000;
    z(1,i) = msgs{i}.Point.Z/1000;
    
    raw_data(i,:) = [z(1,i), -x(1,i), -y(1,i), msgs{i}.Header.Stamp.Nsec/1000000000];
end

%remove all the zeros from the points
points = [];
for i = 1:3
    pos = raw_data(:,i);
    pos(pos==0)=[];
    points(:,i) = pos;
end

%plot3(z, -x, -y);
title('3D PLane');
xlabel('X')
ylabel('Y')
zlabel('Z')
plot3(points(:,1), points(:,2), points(:,3),'o')
trplot(transl(0,0,0));
% title('YZ PLane');
% xlabel('Y')
% ylabel('Z')
% for i = 1:size(points,1)
%     plot(points(i,2), points(i,3), 'x');
%     %pause(1)
% end

hold off

% 
% coefficients = polyfit(x, y, 1);
% xFit = linspace(min(x), max(x), 1000);
% yFit = polyval(coefficients , xFit);
% hold on;
% plot(xFit, yFit, 'r-', 'LineWidth', 2);
% grid on;

%% show 2D line at XZ plane and YZ plane

figure(2)
hold on

subplot(1,2,1);
axis([0 1 0 1])
title('XZ PLane');
plot(points(:,1)',points(:,3)')

subplot(1,2,2);
axis([0 1 0 1])
title('YZ PLane');
plot(points(:,2)',points(:,3)')

hold off

%% Line of best fit

% figure(1)
% [x_p, y_p] = prepareCurveData(points(:,1),points(:,3));
% f = fit(x_p, y_p,'smoothingspline');
% plot(f,x_p, y_p);
% f(0.2)

figure(2)
tic
clf
title('XZ Plane')
f = fit(points(20:25,1),points(20:25,3),'poly2');
plot(f,points(:,1),points(:,3))
toc

%plot(f)

% figure(3)
% title('YZ Plane')
% f = fit(points(22:27,2),points(22:27,3),'poly2');
% plot(f,points(:,2),points(:,3))

% p = polyfit(points(:,1),points(:,3),7);
% y_poly = 0:0.1:5;
% x_poly = polyval(p, y_poly);
% figure(3)
% plot(x_poly, y_poly)




