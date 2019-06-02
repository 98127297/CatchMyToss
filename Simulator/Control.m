function varargout = Control(varargin)
% CONTROL MATLAB code for Control.fig
%      CONTROL, by itself, creates a new CONTROL or raises the existing
%      singleton*.
%
%      H = CONTROL returns the handle to a new CONTROL or the handle to
%      the existing singleton*.
%
%      CONTROL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONTROL.M with the given input arguments.
%
%      CONTROL('Property','Value',...) creates a new CONTROL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Control_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Control_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Control

% Last Modified by GUIDE v2.5 02-Jun-2019 12:29:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Control_OpeningFcn, ...
                   'gui_OutputFcn',  @Control_OutputFcn, ...
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


% --- Executes just before Control is made visible.
function Control_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Control (see VARARGIN)

% Choose default command line output for Control
handles.output = hObject;
% Initialising & Plotting the UR3
handles.robot = UR3();
handles.robot.workspace = [-1 1 -1 1 0 1];
handles.robot.PlotAndColourRobot();
% Global Variables
handles.tolerance = 0.05;
handles.move = 0.01;

% Update handles structure
guidata(hObject, handles);



% UIWAIT makes Control wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Control_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function q0_Callback(hObject, eventdata, handles)
% hObject    handle to q0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% r = the robot's handle
r = handles.robot;
% Obtain the new joint values
j0 = get(hObject,'Value')*pi/180;
j1 = handles.q1.Value*pi/180;
j2 = handles.q2.Value*pi/180;
j3 = handles.q3.Value*pi/180;
j4 = handles.q4.Value*pi/180;
j5 = handles.q5.Value*pi/180;
% Make it into an array
q = [j0 j1 j2 j3 j4 j5];
% Plot the new joint values
r.model.animate(q);


% --- Executes during object creation, after setting all properties.
function q0_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q0 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q1_Callback(hObject, eventdata, handles)
% hObject    handle to q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
% r = the robot's handle
r = handles.robot;
% Obtain the new joint values
j0 = handles.q0.Value*pi/180;
j1 = get(hObject,'Value')*pi/180;
j2 = handles.q2.Value*pi/180;
j3 = handles.q3.Value*pi/180;
j4 = handles.q4.Value*pi/180;
j5 = handles.q5.Value*pi/180;
% Make it into an array
q = [j0 j1 j2 j3 j4 j5];
% Plot the new joint values
r.model.animate(q);



% --- Executes during object creation, after setting all properties.
function q1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q2_Callback(hObject, eventdata, handles)
% hObject    handle to q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% r = the robot's handle
r = handles.robot;
% Obtain the new joint values
j0 = handles.q0.Value*pi/180;
j1 = handles.q1.Value*pi/180;
j2 = get(hObject,'Value')*pi/180;
j3 = handles.q3.Value*pi/180;
j4 = handles.q4.Value*pi/180;
j5 = handles.q5.Value*pi/180;
% Make it into an array
q = [j0 j1 j2 j3 j4 j5];
% Plot the new joint values
r.model.animate(q);


% --- Executes during object creation, after setting all properties.
function q2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q3_Callback(hObject, eventdata, handles)
% hObject    handle to q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% r = the robot's handle
r = handles.robot;
% Obtain the new joint values
j0 = handles.q0.Value*pi/180;
j1 = handles.q1.Value*pi/180;
j2 = handles.q2.Value*pi/180;
j3 = get(hObject,'Value')*pi/180;
j4 = handles.q4.Value*pi/180;
j5 = handles.q5.Value*pi/180;
% Make it into an array
q = [j0 j1 j2 j3 j4 j5];
% Plot the new joint values
r.model.animate(q);


% --- Executes during object creation, after setting all properties.
function q3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q4_Callback(hObject, eventdata, handles)
% hObject    handle to q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% r = the robot's handle
r = handles.robot;
% Obtain the new joint values
j0 = handles.q0.Value*pi/180;
j1 = handles.q1.Value*pi/180;
j2 = handles.q2.Value*pi/180;
j3 = handles.q3.Value*pi/180;
j4 = get(hObject,'Value')*pi/180;
j5 = handles.q5.Value*pi/180;
% Make it into an array
q = [j0 j1 j2 j3 j4 j5];
% Plot the new joint values
r.model.animate(q);

% --- Executes during object creation, after setting all properties.
function q4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function q5_Callback(hObject, eventdata, handles)
% hObject    handle to q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% r = the robot's handle
r = handles.robot;
% Obtain the new joint values
j0 = handles.q0.Value*pi/180;
j1 = handles.q1.Value*pi/180;
j2 = handles.q2.Value*pi/180;
j3 = handles.q3.Value*pi/180;
j4 = handles.q4.Value*pi/180;
j5 = get(hObject,'Value')*pi/180;
% Make it into an array
q = [j0 j1 j2 j3 j4 j5];
% Plot the new joint values
r.model.animate(q);

% --- Executes during object creation, after setting all properties.
function q5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in left.
function left_Callback(hObject, eventdata, handles)
% hObject    handle to left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% r = the robot's handle
r = handles.robot;
% Obtain current joint positions
q0 = r.model.getpos()
% Make handles easier to use
tolerance = handles.tolerance
step = handles.move

% While loop to find the best joint positions
while(1) 
    %Get current end effector's transform
    T = r.model.fkine(q0)
    % Save current transfrom
    I = T;
    % Make new transform
    T(2,4) = T(2,4)-step;
    % Obtain Cartesian Trajectory of new and previous transformations
    Ts = ctraj(T,I,2)
    % Get new joint positions
    [q1,error] = r.model.ikcon(Ts,q0)
    % Check if the joint positions are within the ideal tolerance
    if error(1) <= tolerance
        %if so, plot and break while loop
        r.model.animate(q1(1,:))
        break;
    end
    %if not, continue until best joint angles are found
    q0 = q1;
    drawnow()
end
% Reset button and update handle
handles.left.Value = 0;
guidata(hObject,handles);

% --- Executes on button press in right.
function right_Callback(hObject, eventdata, handles)
% hObject    handle to right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% r = the robot's handle
r = handles.robot;
% Obtain current joint positions
q0 = r.model.getpos()
% Make handles easier to use
tolerance = handles.tolerance
step = handles.move

% While loop to find the best joint positions
while(1) 
    %Get current end effector's transform
    T = r.model.fkine(q0)
    % Save current transfrom
    I = T;
    % Make new transform
    T(2,4) = T(2,4)+step;
    % Obtain Cartesian Trajectory of new and previous transformations
    Ts = ctraj(T,I,2)
    % Get new joint positions
    [q1,error] = r.model.ikcon(Ts,q0)
    % Check if the joint positions are within the ideal tolerance
    if error(1) <= tolerance
        %if so, plot and break while loop
        r.model.animate(q1(1,:))
        break;
    end
    %if not, continue until best joint angles are found
    q0 = q1;
    drawnow()
end
% Reset button and update handle
handles.right.Value = 0;
guidata(hObject,handles);

% --- Executes on button press in forward.
function forward_Callback(hObject, eventdata, handles)
% hObject    handle to forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% r = the robot's handle
r = handles.robot;
% Obtain current joint positions
q0 = r.model.getpos()
% Make handles easier to use
tolerance = handles.tolerance
step = handles.move

% While loop to find the best joint positions
while(1) 
    %Get current end effector's transform
    T = r.model.fkine(q0)
    % Save current transfrom
    I = T;
    % Make new transform
    T(1,4) = T(1,4)-step;
    % Obtain Cartesian Trajectory of new and previous transformations
    Ts = ctraj(T,I,2)
    % Get new joint positions
    [q1,error] = r.model.ikcon(Ts,q0)
    % Check if the joint positions are within the ideal tolerance
    if error(1) <= tolerance
        %if so, plot and break while loop
        r.model.animate(q1(1,:))
        break;
    end
    %if not, continue until best joint angles are found
    q0 = q1;
    drawnow()
end
% Reset button and update handle
handles.forward.Value = 0;
guidata(hObject,handles);


% --- Executes on button press in back.
function back_Callback(hObject, eventdata, handles)
% hObject    handle to back (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% r = the robot's handle
r = handles.robot;
% Obtain current joint positions
q0 = r.model.getpos()
% Make handles easier to use
tolerance = handles.tolerance
step = handles.move

% While loop to find the best joint positions
while(1) 
    %Get current end effector's transform
    T = r.model.fkine(q0)
    % Save current transfrom
    I = T;
    % Make new transform
    T(1,4) = T(1,4)+step;
    % Obtain Cartesian Trajectory of new and previous transformations
    Ts = ctraj(T,I,2)
    % Get new joint positions
    [q1,error] = r.model.ikcon(Ts,q0)
    % Check if the joint positions are within the ideal tolerance
    if error(1) <= tolerance
        %if so, plot and break while loop
        r.model.animate(q1(1,:))
        break;
    end
    %if not, continue until best joint angles are found
    q0 = q1;
    drawnow()
end
% Reset button and update handle
handles.left.Value = 0;
guidata(hObject,handles);

% --- Executes on button press in up.
function up_Callback(hObject, eventdata, handles)
% hObject    handle to up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% r = the robot's handle
r = handles.robot;
% Obtain current joint positions
q0 = r.model.getpos()
% Make handles easier to use
tolerance = handles.tolerance
step = handles.move

% While loop to find the best joint positions
while(1) 
    %Get current end effector's transform
    T = r.model.fkine(q0)
    % Save current transfrom
    I = T;
    % Make new transform
    T(3,4) = T(3,4)+step;
    % Obtain Cartesian Trajectory of new and previous transformations
    Ts = ctraj(T,I,2)
    % Get new joint positions
    [q1,error] = r.model.ikcon(Ts,q0)
    % Check if the joint positions are within the ideal tolerance
    if error(1) <= tolerance
        %if so, plot and break while loop
        r.model.animate(q1(1,:))
        break;
    end
    %if not, continue until best joint angles are found
    q0 = q1;
    drawnow()
end
% Reset button and update handle
handles.up.Value = 0;
guidata(hObject,handles);

% --- Executes on button press in down.
function down_Callback(hObject, eventdata, handles)
% hObject    handle to down (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% r = the robot's handle
r = handles.robot;
% Obtain current joint positions
q0 = r.model.getpos()
% Make handles easier to use
tolerance = handles.tolerance
step = handles.move

% While loop to find the best joint positions
while(1) 
    %Get current end effector's transform
    T = r.model.fkine(q0)
    % Save current transfrom
    I = T;
    % Make new transform
    T(3,4) = T(3,4)-step;
    % Obtain Cartesian Trajectory of new and previous transformations
    Ts = ctraj(T,I,2)
    % Get new joint positions
    [q1,error] = r.model.ikcon(Ts,q0)
    % Check if the joint positions are within the ideal tolerance
    if error(1) <= tolerance
        %if so, plot and break while loop
        r.model.animate(q1(1,:))
        break;
    end
    %if not, continue until best joint angles are found
    q0 = q1;
    drawnow()
end
% Reset button and update handle
handles.down.Value = 0;
guidata(hObject,handles);
