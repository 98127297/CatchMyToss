function varargout = mainGUI(varargin)
% MAINGUI MATLAB code for mainGUI.fig
%      MAINGUI, by itself, creates a new MAINGUI or raises the existing
%      singleton*.
%
%      H = MAINGUI returns the handle to a new MAINGUI or the handle to
%      the existing singleton*.
%
%      MAINGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAINGUI.M with the given input arguments.
%
%      MAINGUI('Property','Value',...) creates a new MAINGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before mainGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to mainGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help mainGUI

% Last Modified by GUIDE v2.5 02-Jun-2019 14:05:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @mainGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @mainGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before mainGUI is made visible.
function mainGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to mainGUI (see VARARGIN)

% Choose default command line output for mainGUI
handles.output = hObject;

% Joint Positions for Throwing
handles.qStart = deg2rad([90,-110,110,-80,-90,0]);
handles.qEnd = deg2rad([90,-110,-20,-170,-90,0]);

% Joint Position for Catching
handles.qCatch = deg2rad([90 0 80 -70 90 0]);

% Initialise the UR3
handles.thrower = UR3();
handles.catcher = UR3();

% Initial Plot of Thrower
thrower = handles.thrower;
thrower.model.base = transl(0,0,0.966);
thrower.toolModelFilename = ['gripper.ply']
thrower.PlotAndColourRobot();
thrower.model.animate(handles.qStart);

%Initial Plot of Catcher
catcher = handles.catcher;
catcher.model.base = transl(0,1.2,0.3);
catcher.toolModelFilename = ['box.ply'];
catcher.PlotAndColourRobot();
catcher.model.animate(handles.qCatch);
axis equal;

% Import the Benches
try delete(bench_h);end;
try delete(smBench_h);end;
bench_h = GetandMovePart('bench.ply',transl(0,-1.2,0.5)*trotz(-pi/2));
smBench_h = GetandMovePart('smallbench.ply',transl(0,1.6,0.1)*trotz(pi/2));

% Object's tranformation
handles.object_tr = transl(1.5,1,0.25);
handles.counter = 1;

% Import Object
try delete(handles.object_h);end;
handles.object_h = GetandMovePart('object.ply',handles.object_tr);

% Import Safety Curtain
[handles.startP,handles.endP] = SafetyCurtain(hObject,handles);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes mainGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function [sP,eP] = SafetyCurtain(hObject, handles)
    %Start and End points of one of the safety curtains
    startP1 = [1 -1 0.4];
    startP2 = [1 -1 0.8];
    startP3 = [1 -1 1.2];
    startP4 = [1 -1 1.6];
    startP5 = [1 -1 2];

    endP1 = [1 2 0.4];
    endP2 = [1 2 0.8];
    endP3 = [1 2 1.2];
    endP4 = [1 2 1.6];
    endP5 = [1 2 2];
    
    % Saving the points for Line Intersection
    sP = startP1;
    eP = endP1;

    % Plotiing the lines
    line1 = plot3([startP1(1),endP1(1)],[startP1(2),endP1(2)],[startP1(3),endP1(3)],'r','LineWidth',1);
    line2 = plot3([startP2(1),endP2(1)],[startP2(2),endP2(2)],[startP2(3),endP2(3)],'r','LineWidth',1);
    line3 = plot3([startP3(1),endP3(1)],[startP3(2),endP3(2)],[startP3(3),endP3(3)],'r','LineWidth',1);
    line4 = plot3([startP4(1),endP4(1)],[startP4(2),endP4(2)],[startP4(3),endP4(3)],'r','LineWidth',1);
    line5 = plot3([startP5(1),endP5(1)],[startP5(2),endP5(2)],[startP5(3),endP5(3)],'r','LineWidth',1);
    hold on;
    
    % Creating the Poles(limits)
    pole1 = plot3([startP1(1),startP5(1)],[startP1(2),startP5(2)],[0,startP5(3)],'k','LineWidth',5);
    pole2 = plot3([endP1(1),endP5(1)],[endP1(2),endP5(2)],[0,endP5(3)],'k','LineWidth',5);
    
    % Start and End points of the other curtain
    startP1 = [-1 -1 0.4];
    startP2 = [-1 -1 0.8];
    startP3 = [-1 -1 1.2];
    startP4 = [-1 -1 1.6];
    startP5 = [-1 -1 2];

    endP1 = [-1 2 0.4];
    endP2 = [-1 2 0.8];
    endP3 = [-1 2 1.2];
    endP4 = [-1 2 1.6];
    endP5 = [-1 2 2];
    
    % Plotting the other curtain
    line1 = plot3([startP1(1),endP1(1)],[startP1(2),endP1(2)],[startP1(3),endP1(3)],'r','LineWidth',1);
    line2 = plot3([startP2(1),endP2(1)],[startP2(2),endP2(2)],[startP2(3),endP2(3)],'r','LineWidth',1);
    line3 = plot3([startP3(1),endP3(1)],[startP3(2),endP3(2)],[startP3(3),endP3(3)],'r','LineWidth',1);
    line4 = plot3([startP4(1),endP4(1)],[startP4(2),endP4(2)],[startP4(3),endP4(3)],'r','LineWidth',1);
    line5 = plot3([startP5(1),endP5(1)],[startP5(2),endP5(2)],[startP5(3),endP5(3)],'r','LineWidth',1);
    hold on;
    
    % Poles of the other safety curtain
    pole1 = plot3([startP1(1),startP5(1)],[startP1(2),startP5(2)],[0,startP5(3)],'k','LineWidth',5);
    pole2 = plot3([endP1(1),endP5(1)],[endP1(2),endP5(2)],[0,endP5(3)],'k','LineWidth',5);

% --- Outputs from this function are returned to the command line.
function varargout = mainGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% Initialise Variables
% Joint Positions for Throwing
qStart_throwing = handles.qStart;
qEnd_throwing = handles.qEnd;

% Joint Position for Catching
qStart_catching = handles.qCatch;

thrower = handles.thrower;
catcher = handles.catcher;

% Transform of the robots
tc = catcher.model.fkine(qStart_catching);
tr = thrower.model.fkine(qStart_throwing);

steps = 100;

set(hObject, 'UserData', false);

% Turn on the Status
handles.indicator.BackgroundColor = [0 1 0];

%% Main Simulation
qMatrix_thrower = mtraj(@lspb,qStart_throwing,qEnd_throwing,steps);
release_ball_step = 70;
q_ball_initial = qMatrix_thrower(release_ball_step,:);
q_ball_inital_T = thrower.model.fkine(q_ball_initial);
X = q_ball_inital_T(1,4);
Y = q_ball_inital_T(2,4);
Z = q_ball_inital_T(3,4);
[y,z] = Projectile(Z,steps);

%Offseting the projectile motion to the end effector
x = zeros(1,steps+1);
for i = 1:steps+1 
    x(i) = X;
end

y = y+Y;
ball = [x' y' z'];

% determine stop location of the ball
for j = 1:size(ball,1)
    if ball(j,3) < 0.1840
        ball_stop_step = j;
        ball_stop_point = ball(j,:);
        break
    end
end

% calculate qMatrix_catching
qEnd_catching_t = transl(ball_stop_point(1,1), ball_stop_point(1,2)+0.05, tc(3,4)+0.05) * trotx(90,'deg');
qEnd_catching = catcher.model.ikcon(qEnd_catching_t, qStart_catching);
qMatrix_catching = mtraj(@lspb,qStart_catching,qEnd_catching,steps);

count = 1;
release_ball = false;

% Plotting the simulation
for t = 1:2*steps
        
    % Testing if the emergency button or the safety curtain has been activated
    drawnow();        %give a chance for interrupts
    need_to_stop = get(handles.start, 'UserData');
    if ~isempty(need_to_stop) && need_to_stop;
        handles.indicator.BackgroundColor = [1 0 0];
        try delete(ball_p);end;
        try delete(ball_h);end;
        break;
    end   
   
    
    if t < steps % plotting the thrower
        thrower.model.animate(qMatrix_thrower(t,:));
        thrower_tr = thrower.model.fkine(qMatrix_thrower(t,:));
        try delete(ball_h);end
        ball_h = GetandMovePart('ball.ply',thrower_tr);
        if t == release_ball_step
            release_ball = true;
        end
    end
    
    if t > steps % plotting the catcher
        catcher.model.animate(qMatrix_catching(t-steps,:));
    end
    
    % plotting the golf ball
    if release_ball == true
        
        % plot position of the ball
        try delete(ball_h);end
        ball_h = GetandMovePart('ball.ply',transl(ball(count,1),ball(count,2),ball(count,3)));
        drawnow()
        count = count+1;
        
        % plot previous trajectory
        if mod(count,10) == 0
            ball_p(count) = plot3(ball(count,1),ball(count,2),ball(count,3), 'ro');
            drawnow()
        end
        
        % stop the ball at a certain height
        if count == ball_stop_step
            release_ball = false;
        end    
    end  
end

%% Projectile function
function [x,z] = Projectile(zOff,steps)
%Projectile Motion Equations
% Initialise Variables
G = 9.81; %m/s^2
angle = pi/4; %degrees
v = 1; %m/s
vX = v*cos(angle);
vZ = v*sin(angle);


%Ball Projectile
xmax = (vX * (vX + sqrt((vZ.^2 + 2 * G * zOff))) / G)    %% Calculates max x distance
xstep = xmax / steps;                        %% Calculates step, always 100 samples
           
x = 0:xstep:xmax;
z = (x * tan(angle) - G/(2*(v.^2)*(cos(angle)).^2)*x.^2) + zOff;


%% Import Part and Update Pose
function part_h = GetandMovePart(part,tr)
    [face,vertex,data] = plyread(part,'tri');

    vertexColours = [data.vertex.red,data.vertex.green,data.vertex.blue]/225;

    hold on;

    part_h = trisurf(face,vertex(:,1),vertex(:,2),vertex(:,3),'FaceVertexCData', vertexColours,'EdgeColor','interp','EdgeLighting','flat');

    partVertexCount = size(vertex,1);

    midPoint = sum(vertex)/partVertexCount;
    pVerts = vertex - repmat(midPoint,partVertexCount,1);

    pose = eye(4);
    pose = pose * tr;

    updatedPoints = [pose * [pVerts,ones(partVertexCount,1)]']';  

    part_h.Vertices = updatedPoints(:,1:3);

    hold on;


% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Notifies the simulation that the button has ben pressed
set(handles.start, 'UserData', true);


% --- Executes during object creation, after setting all properties.
function object_CreateFcn(hObject, eventdata, handles)
% hObject    handle to object (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function object_Callback(hObject, eventdata, handles)
% hObject    handle to object (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% Make it easier to use
object_h = handles.object_h;
object_tr = handles.object_tr;
counter = handles.counter;
% Moving object cube
new_tr = object_tr;
new_tr(1,4) = object_tr(1,4) - handles.object.Value;
try delete(object_h);end;
object_h(counter) = GetandMovePart('object.ply',new_tr);

if (new_tr - 0.25) <= 1 %safety curtain location
    set(handles.start, 'UserData', true);
end

%update handle
handles.counter = counter + 1;
handles.object_h = object_h(counter);
guidata(hObject,handles);



% --- Executes on button press in indicator.
function indicator_Callback(hObject, eventdata, handles)
% hObject    handle to indicator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of indicator


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over indicator.
function indicator_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to indicator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
