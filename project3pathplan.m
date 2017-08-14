function varargout = project3pathplan(varargin)
% PROJECT3PATHPLAN MATLAB code for project3pathplan.fig
%      PROJECT3PATHPLAN, by itself, creates a new PROJECT3PATHPLAN or raises the existing
%      singleton*.
%
%      H = PROJECT3PATHPLAN returns the handle to a new PROJECT3PATHPLAN or the handle to
%      the existing singleton*.
%
%      PROJECT3PATHPLAN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PROJECT3PATHPLAN.M with the given input arguments.
%
%      PROJECT3PATHPLAN('Property','Value',...) creates a new PROJECT3PATHPLAN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before project3pathplan_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to project3pathplan_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help project3pathplan

% Last Modified by GUIDE v2.5 11-Dec-2016 00:47:17

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @project3pathplan_OpeningFcn, ...
                   'gui_OutputFcn',  @project3pathplan_OutputFcn, ...
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

% --- Executes just before project3pathplan is made visible.
function project3pathplan_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to project3pathplan (see VARARGIN)

% Choose default command line output for project3pathplan
% -------------------------------------------------------------------------
% Defining handles structure for MyBot and it's Joints
% -------------------------------------------------------------------------
handles.output = hObject; %handles for GUI Output..
handles.mybot=[];% Handles of robot...
handles.ja=0; % initiate handles for joint angle 1
handles.jb=0; % initiate handles for joint angle 2
handles.jc=0; % initiate handles for joint angle 3
handles.b=0; 
handles.pp = PathPlanning(); % Handles structure to store class 'PathPlanning'
handles.start = 0; % handles structure to get start point
handles.end = 0; % handles structure variable to store end point
handles.ppalg = 0; % handles structure to store Path Planning algorithm
handles.trajgen = 0; % handles structure to store trajectory generation algorithm..
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes project3pathplan wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = project3pathplan_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% -------------------------------------------------------------------------
% Creating MyBot's Link in GUIDE using D-H parameters of MyBot
% All Dimensions are in 'Meter', All angles are in 'Radians'
% -------------------------------------------------------------------------
L(1) = Link('revolute', 'd', 0, 'a', .1035582, 'alpha', 0, 'offset', .1303761, 'qlim', [0 3.665191429188092]); % Link 1 of robot..
L(2) = Link('revolute', 'd', .024384, 'a', .08287, 'alpha', pi/2, 'offset', .1717404, 'qlim', [0 3.665191429188092]); % link 2 of the robot..
L(3) = Link('revolute', 'd', .016435667, 'a', 0, 'alpha', 0, 'offset', 0, 'qlim', [0 2.879793265790644]); % Link 3 of the Robot..
T=(r2t(rotx(-pi/2)))*transl([0 0 -.09]);% Tool of the ee of the robot..

% Creating Three-link MyBot using D-H Parameters, one set per joint
handles.mybot = SerialLink(L, 'tool', T, 'name', 'mybot');

% Plotting MyBot
handles.mybot.plot([handles.ja, handles.jb, handles.jc]); % Plotting  the robot with joint angle handles as variable..
zoom on; % zoom on...

guidata(hObject,handles); % update handles structure..

% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.ja=get(hObject, 'Value');
t=handles.mybot.fkine([handles.ja, handles.jb, handles.jc]);
p=t(1:3,4);
hold on;
plot3(p(1),p(2),p(3),'--bs');
handles.mybot.plot([handles.ja, handles.jb, handles.jc]);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.jb=get(hObject, 'Value');
t=handles.mybot.fkine([handles.ja, handles.jb, handles.jc]);
p=t(1:3,4);
hold on;
plot3(p(1),p(2),p(3),'--rs');
handles.mybot.plot([handles.ja, handles.jb, handles.jc]);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.jc=get(hObject, 'Value');
handles.mybot.plot([handles.ja, handles.jb, handles.jc]);
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in reach_envelop.
function reach_envelop_Callback(hObject, eventdata, handles)
% hObject    handle to reach_envelop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
for x=0:.1:3.577924966588375
    for y=0:.1:3.577924966588375
        v=handles.mybot.fkine([x,y,0]);
        u=v(1:3,4);
        hold on;
        handles.mybot.plot([x,y,0]);
        plot3(u(1),u(2),u(3),'--rs');
    end
end
hold off


% --- Executes on button press in start_1_1.
function start_1_1_Callback(hObject, eventdata, handles)
% hObject    handle to start_1_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of start_1_1


% --- Executes on button press in start_1_2.
function start_1_2_Callback(hObject, eventdata, handles)
% hObject    handle to start_1_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of start_1_2

% --- Executes on button press in start_1_3.
function start_1_3_Callback(hObject, eventdata, handles)
% hObject    handle to start_1_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of start_1_3

% --- Executes on button press in start_2_1.
function start_2_1_Callback(hObject, eventdata, handles)
% hObject    handle to start_2_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of start_2_1

% --- Executes on button press in start_2_2.
function start_2_2_Callback(hObject, eventdata, handles)
% hObject    handle to start_2_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of start_2_2

% --- Executes on button press in start_2_3.
function start_2_3_Callback(hObject, eventdata, handles)
% hObject    handle to start_2_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of start_2_3

% --- Executes on button press in start_3_1.
function start_3_1_Callback(hObject, eventdata, handles)
% hObject    handle to start_3_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of start_3_1

% --- Executes on button press in start_3_2.
function start_3_2_Callback(hObject, eventdata, handles)
% hObject    handle to start_3_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of start_3_2

% --- Executes on button press in start_3_3.
function start_3_3_Callback(hObject, eventdata, handles)
% hObject    handle to start_3_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of start_3_3


% --- Executes on button press in end_1_1.
function end_1_1_Callback(hObject, eventdata, handles)
% hObject    handle to end_1_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of end_1_1

% --- Executes on button press in end_1_2.
function end_1_2_Callback(hObject, eventdata, handles)
% hObject    handle to end_1_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of end_1_2

% --- Executes on button press in end_1_3.
function end_1_3_Callback(hObject, eventdata, handles)
% hObject    handle to end_1_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of end_1_3

% --- Executes on button press in end_2_1.
function end_2_1_Callback(hObject, eventdata, handles)
% hObject    handle to end_2_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of end_2_1

% --- Executes on button press in end_2_2.
function end_2_2_Callback(hObject, eventdata, handles)
% hObject    handle to end_2_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of end_2_2

% --- Executes on button press in end_2_3.
function end_2_3_Callback(hObject, eventdata, handles)
% hObject    handle to end_2_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of end_2_3

% --- Executes on button press in end_3_1.
function end_3_1_Callback(hObject, eventdata, handles)
% hObject    handle to end_3_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of end_3_1

% --- Executes on button press in end_3_2.
function end_3_2_Callback(hObject, eventdata, handles)
% hObject    handle to end_3_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of end_3_2

% --- Executes on button press in end_3_3.
function end_3_3_Callback(hObject, eventdata, handles)
% hObject    handle to end_3_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of end_3_3


% --- Executes on button press in d_star_algorithm.
function d_star_algorithm_Callback(hObject, eventdata, handles)
% hObject    handle to d_star_algorithm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of d_star_algorithm


% --- Executes on button press in prm_algorithm.
function prm_algorithm_Callback(hObject, eventdata, handles)
% hObject    handle to prm_algorithm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of prm_algorithm


% --- Executes on button press in no_interpolation.
function no_interpolation_Callback(hObject, eventdata, handles)
% hObject    handle to no_interpolation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of no_interpolation


% --- Executes on button press in mtraj_tpoly.
function mtraj_tpoly_Callback(hObject, eventdata, handles)
% hObject    handle to mtraj_tpoly (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of mtraj_tpoly


% --- Executes on button press in mtraj_lspb.
function mtraj_lspb_Callback(hObject, eventdata, handles)
% hObject    handle to mtraj_lspb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of mtraj_lspb


% --- Executes when selected object is changed in select_start_point.
function select_start_point_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in select_start_point 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch get(eventdata.NewValue,'Tag') % Get Tag of selected object.
    case 'start_1_1' % changes on radio button press start 11
        display('Start Point : Start 11') %  displays value..
        handles.start = 11; % Stores value in handles.start to send to Pathplanning Class
    case 'start_1_2' % changes on radio button press start 12
        display('Start Point : Start 12') %  displays value..
        handles.start = 12; % Stores value in handles.start to send to Pathplanning Class
    case 'start_1_3' % changes on radio button press start 13
        display('Start Point : Start 13') %  displays value..
        handles.start = 13; % Stores value in handles.start to send to Pathplanning Class
    case 'start_2_1' % changes on radio button press start 21
        display('Start Point : Start 21') %  displays value..
        handles.start = 21; % Stores value in handles.start to send to Pathplanning Class
    case 'start_2_2' % changes on radio button press start 22
        display('Start Point : Start 22') %  displays value..
        handles.start = 22; % Stores value in handles.start to send to Pathplanning Class
    case 'start_2_3' % changes on radio button press start 23
        display('Start Point : Start 23') %  displays value..
        handles.start = 23; % Stores value in handles.start to send to Pathplanning Class
    case 'start_3_1' % changes on radio button press start 31
        display('Start Point : Start 31') %  displays value..
        handles.start = 31; % Stores value in handles.start to send to Pathplanning Class
    case 'start_3_2' % changes on radio button press start 32
        display('Start Point : Start 32') %  displays value..
        handles.start = 32; % Stores value in handles.start to send to Pathplanning Class
    case 'start_3_3' % changes on radio button press start 33
        display('Start Point : Start 33') %  displays value..
        handles.start = 33; % Stores value in handles.start to send to Pathplanning Class
end
% Update handles structure
guidata(hObject, handles);

% --- Executes when selected object is changed in select_end_point.
function select_end_point_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in select_end_point 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch get(eventdata.NewValue,'Tag') % Get Tag of selected object.
    case 'end_1_1' % changes on radio button press end 11
        display('End Point : End 11') %  displays value..
        handles.end = 11; % Stores value in handles.start to send to Pathplanning Class
    case 'end_1_2' % changes on radio button press end 12
        display('End Point : End 12') %  displays value..
        handles.end = 12; % Stores value in handles.start to send to Pathplanning Class
    case 'end_1_3' % changes on radio button press end 13
        display('End Point : End 13') %  displays value..
        handles.end = 13; % Stores value in handles.start to send to Pathplanning Class
    case 'end_2_1' % changes on radio button press end 21
        display('End Point : End 21') %  displays value..
        handles.end = 21; % Stores value in handles.start to send to Pathplanning Class
    case 'end_2_2' % changes on radio button press end 22
        display('End Point : End 22') %  displays value..
        handles.end = 22; % Stores value in handles.start to send to Pathplanning Class
    case 'end_2_3' % changes on radio button press end 23
        display('End Point : End 23') %  displays value..
        handles.end = 23; % Stores value in handles.start to send to Pathplanning Class
    case 'end_3_1' % changes on radio button press end  31
        display('End Point : End 31') %  displays value..
        handles.end = 31; % Stores value in handles.start to send to Pathplanning Class
    case 'end_3_2' % changes on radio button press end 32
        display('End Point : End 32') %  displays value..
        handles.end = 32; % Stores value in handles.start to send to Pathplanning Class
    case 'end_3_3' % changes on radio button press end 33
        display('End Point : End 33') %  displays value..
        handles.end = 33; % Stores value in handles.start to send to Pathplanning Class
end
% Update handles structure
guidata(hObject, handles);

% --- Executes when selected object is changed in select_path_plan_algorithm.
function select_path_plan_algorithm_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in select_path_plan_algorithm 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch get(eventdata.NewValue,'Tag') % Get Tag of selected object.
    case 'd_star_algorithm' % when D* algorithm selection button i pressed...
        display('D* Algorithm Selected')
        handles.ppalg = 1; % store value to exchange with class
    case 'prm_algorithm' % when PRM algorithm selection button i pressed...
        display('PRM Algorithm Selected')
        handles.ppalg = 2; % store value to exchange with class
end
% Update handles structure
guidata(hObject, handles);
    
% --- Executes when selected object is changed in traj_gen_method.
function traj_gen_method_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in traj_gen_method 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%no_interpolation
switch get(eventdata.NewValue,'Tag') % Get Tag of selected object.
    case 'no_interpolation' % When no interpolation is selected...
        display('No Interpolation Method Selected...!!')
        handles.trajgen = 1; % Store the value in trajgen..
    case 'mtraj_tpoly' % When mtraj@tpoly interpolation is selected...
        display('mtraj with tpoly selected...')
        handles.trajgen = 2; % Store the value in trajgen..
    case 'mtraj_lspb'% When mtraj @ lspb interpolation is selected...
        display('mtraj with lspb selected...')
        handles.trajgen = 3; % Store the value in trajgen..
    case 'only_tpoly' % When only tpoly interpolation is selected...
        display('Only tpoly selected...')
        handles.trajgen = 4; % Store the value in trajgen..
    case 'only_lspb' % When only lspb interpolation is selected...
        display('Only lspb selected...')
        handles.trajgen = 5; % Store the value in trajgen..
    case 'only_spline' % When only spline interpolation is selected...
        display('Only spline selected...')
        handles.trajgen = 6; % Store the value in trajgen..
end
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in start_plotting_tajectory.
function start_plotting_tajectory_Callback(hObject, eventdata, handles)
% hObject    handle to start_plotting_tajectory (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if handles.ppalg == 1 % if handles.ppalg value is one,
    % D* algorithm is used...
    handles.pp.dStarAlgo(handles.start, handles.end, handles.trajgen); % value is given to class Pathplanning
elseif handles.ppalg == 2 % if handles.ppalg value is two.. 
    % PRM Algorithm is used..
    handles.pp.prmAlgo(handles.start, handles.end, handles.trajgen); % value is given to class..
end


% --- Executes on button press in robot_initiate.
function robot_initiate_Callback(hObject, eventdata, handles)
% hObject    handle to robot_initiate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.pp = handles.pp.connectArd; % updating handles.pp to connect arduino function(obj)
handles.pp = handles.pp.attachServo; % updating handles.pp to attach sevo function(obj)
handles.pp = handles.pp.createBot; % updating handles.pp to createbot function(obj)
handles.pp = handles.pp.bring2CentrPos; % updating handles.pp to Bring to centerpos function(obj)
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in draw_exclusion.
function draw_exclusion_Callback(hObject, eventdata, handles)
% hObject    handle to draw_exclusion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.pp = handles.pp.DrawExclusion; % updates handles.pp structure with DrawExclusion(obj) and draws exclusion
% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in only_tpoly.
function only_tpoly_Callback(hObject, eventdata, handles)
% hObject    handle to only_tpoly (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of only_tpoly


% --- Executes on button press in only_lspb.
function only_lspb_Callback(hObject, eventdata, handles)
% hObject    handle to only_lspb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of only_lspb


% --- Executes on button press in only_spline.
function only_spline_Callback(hObject, eventdata, handles)
% hObject    handle to only_spline (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of only_spline
