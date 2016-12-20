%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                Pipe Inspection using SEA Snake Robot                    %
%                                           by Donghyung Kim, 2016.12.15  %
% API:           sdk0.5-rev1424                                           %
% Tool:          Hebi Plotter                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc

% add the paths
addpath(genpath(pwd));

%% Initialize the joystick
% create the joystick object
joy           = vrjoystick(1);  
joyError      = 0.12;               % anything under this value is considered error (L/R analog stick)

% read the joystick and declare its previous value
[axes, buttons, povs] = read( joy ); 
buttons_prev = buttons;

% Joystick Button Press Counter
JoyCnt_Vel = JoyButtonCnt;          % Counter for Velocity Control
JoyCnt_MagLat = JoyButtonCnt;       % Counter for Lateral Magnitude Control
JoyCnt_MagDor = JoyButtonCnt;       % Counter for Dorsal Magnitude Control
             
% Gain parameter for joystick control
VelGain = 0.05;                     % Velocity gain
MagLatGain = 0.05;                  % Lateral magnitude
MagDorGain = 0.05;                  % Dorsal magnitude


%% Initialize the SEA snake robot (simulation or experiment)

% Show the user selection menu and get the keyboard input
fprintf('--------  Pipe Inspection using SEA Snake Robot  ----------\n');
fprintf(' API:	sdk0.5-rev1424  \nTool:	Hebi Plotter \n');
fprintf('-----------------------------------------------------------\n');


% Get the user selection, simulation or experiment
simQ = GetUserSelect_Sim_or_Exp();    % -> simQ is 'true' for the simulation and 'false' for the experiment
                                      % -> FOR THE EXPERIMENT, MAKE SURE THE SEA SNAKE IS CONNECTED!
% In case of simulation
if simQ
    initializeSEASnake_Sim();

    % initialize the HebiPlotter when user select the simulation
	plt = HebiPlotter('resolution', 'low', 'frame', 'VC');  

	% initial joint angle for the robot
    SEA_angles = 0.5*rand(1,numModules);    % For simulation, give random initial pose
    SEA_angles_prev = SEA_angles;   
    
% In case of experiment
else
    initializeSEASnake();
    
    fprintf('You have %d SEA modules \n', snakeData.num_modules);
    
    % initialize the command structure
    cmd = CommandStruct(); 
    fbk  = snake.getNextFeedback();
    
    % set feedback rate
    snake.setFeedbackFrequency(100);    % Unit: Hz
    
    % set the gain
    SetGains_GroundLocomotion();            % -> use the gains for the ground locomotion since this mode is the default one
    
	% initial joint angle for the robot    
	SEA_angles = snake.getNextFeedback.position;    % For experiment, use the current pose as the initial pose
    SEA_angles_prev = SEA_angles; 
end

% Initialize constant snake modulation, such as the module length
sm.m = snakeData.moduleLen;


%% Initialize the parameter for the outer-pipe inspection

% Initialize the radius and pitch of the rolling helix (Unit: m)
radi.init = 1.2;                    % Initial (or maximum) value of radius of the helix  (Unit: m)
radi.min_ClimbPole = 0.035;         % Minimum value of radius of the helix during the pole climbing (Unit: m)
radi.des_Roll_between_Pipes = 0.025;% Desired value for r  (Unit: m)  --> rolling between two pipes

PitchLen_init = 0.085;              % initial value of pitch of the helix (Unit: m)
pitch.init = PitchLen_init/(2*pi);
pitch.max = 0.0500;                 % This is the limit on 'p' for rolling between two pipes

% Gains for compliance curvature (CC)
sm.TssDes_CC = 5.3;                 % Desired square-sum of joint torques for stable pole climbing
sm.Pgain_CC = 0.008;                % Gain value

% Offset value of iternal time Ot onlt for s only when the robot shift its body from pipe to pipe during crawing
Ot_offset_ShiftPipe = 0;
        

%% initialize the user's operation parameters

% Operation mode
OperMode = 0;           % 0: Ground Locomotion, 1: Outer-pipe Inspection

% Flags to enable/disable the sub-functions of operation mode
EnableSubFcn.HeadLook_GL = false;
EnableSubFcn.HeadLook_OPI = false;
EnableSubFcn.PipeCraw_OPI = false;
EnableSubFcn.RotInPlace_OPI = false;
EnableSubFcn.ShiftRtoP_OPI = false;

% Boolean that determines whether the  program is running or terminated.
bTerminate	= false;	% The on/off button for this program is both buttons 9 and 10 on the controller.

% Status of previous transition between two different gaits / counter for delaying joystick input right after the transition
bTransHappened = false;
JoyDelayCnt = 1;
bTransDoneJoyReady = false;


%% Set time parameters

% Pre-defined time interval for the simulation
if simQ
    dt = pi/160;            % Time interval (sec)   -> pi/160 is regarded as typical value for experiment
% Get feedback'lastTime' time for the experiment
else
	lastTime = fbk.time;
end

% Internal time for gait equation
Ot = 0;         % Internal time for the gait equation (sec)
sm.Ot = Ot;     % Ot is also required for snake modulation or sm

Ot_RiP = 0;     % Internal time only for the rotating in place gait 

%% Main code starts here
LogData = false; %true;

if ((~simQ)&&LogData)
    snake.startLog();
end

fprintf('\nPush both buttons 9 and 10 to terminate the program\n\n');
disp('[Ground Locomotion]')

while (~bTerminate)
    %% (experiment) get feedback and time interval
    if ~simQ
        % Read sensor feedback
        fbk = snake.getNextFeedback(fbk);
        if isempty(fbk)
            disp('fbk is empty!')
        end
        
        % Calculate the time interval (sec)
        dt = (fbk.time - lastTime);
        lastTime = fbk.time;
        
        %Tss = GetSSofTorque(fbk, numModules);   % dislay the square-sum of joint torques

    else
        fbk = [];
    end
    
    %% Delay the joystick input until the transition motion is done
    %   -> This is to prevent the wrong transition motion which is continuously
    %   repeated when user keep press on the D-pad button
    if bTransHappened               % If the transition between two different gaits was happened at the previous iteration,
        bTransHappened = false;     % then flip it (false)
        % disp('Translation was happened');
        JoyDelayCnt = 1;            % and reset the counter as 1
    end
    JoyDelayCnt = JoyDelayCnt + 1;

    if JoyDelayCnt > 15     % for 100Hz
        bTransDoneJoyReady = true;
    else
        bTransDoneJoyReady = false;
    end
    
    %% Read joystick and declare button counter
    [axes, buttons, povs] = read( joy );    % axes, buttons, povs are the data that the joystick will return
    bTerminate = buttons(9)&buttons(10);     % both buttons 9 & 10 terminate the program
    
    for J_id = 1:4  % apply dead zone to the analog joysticks
        axes(J_id) = DeadzoneJoyErr(axes(J_id), joyError);
    end
    
    % Update the Joystick button counters
    JoyCnt_Vel.Update(axes, buttons, povs);
    JoyCnt_MagLat.Update(axes, buttons, povs);
    JoyCnt_MagDor.Update(axes, buttons, povs);
    
    
    %% Change the operation mode with joystick inputs and initialize the related parameters
    %  - Button 10                    ->  Enable 'Ground Locomotion'
    %  - Button 4                     ->  Enable 'Outer-pipe Inspection'
    
    % Enable 'Ground Locomotion' operation mode
    if ((buttons(10))&&(~buttons_prev(10)))&&(~buttons(9))
        OperMode = 0;
        disp('[Ground Locomotion]')
        
        % Disable the sub-function
        EnableSubFcn.HeadLook_GL = false;    
        
        % Set the gains for the ground locomotion mode
        if ~simQ
            SetGains_GroundLocomotion();
        end
        
	% Enable 'Outer-pipe Inspection' operation mode
    elseif ((buttons(4))&&(~buttons_prev(4)))
        OperMode = 1;
        disp('[Outer-pipe Inspection: Pole Climbing] Select the direction of pole')
   
        % Get user's input of the direction of pole (left or right)
        %  -> wait until the user inputs the direction of pole with respect to the robot's heading direction
        GetUserInput_DirectionOfPole();
        
        % Initialize the snake modulation related to the rolling helix
        sm.r = radi.init;
        sm.p = PitchLen_init/(2*pi);
        
        % Disable the sub-function when user starts 'outer-pipe insepection' operation mode
        EnableSubFcn.HeadLook_OPI = false;
        EnableSubFcn.PipeCraw_OPI = false;
        EnableSubFcn.RotInPlace_OPI = false;
        EnableSubFcn.ShiftRtoP_OPI = false;
        
        % Set the gains for the outer pipe inspection mode
        if ~simQ
            SetGains_OuterPipeInspection();
        end
    end
    
    
    %% Update the speed / magnitute gain for the wave (joystick)
    % Here, compute the gain factors 'VelFactor','MagFact' to manually control the speed and the magnitude of wave using the joystick
    
    % Change the speed with buttons 7 and 8
    % - Compute the velocity gain factor, VelFactor to control the speed of wave
    % - Button push counter with two buttons; one decrease and the other increase the number of button presses 
	JoyCnt_Vel.CntWithTwoButtons(7, 8, -15, 15);
    VelFactor = VelGain*JoyCnt_Vel.Count;

    % Change the magnitude with joystick's buttons (1 to 4)
    % - Compute the magnitude gain factor, MagLatFactor / MagDorFactor to control the magnitude of wave
    % - 1) magnitude factor of lateral joint wave
    JoyCnt_MagLat.CntWithTwoButtons(1, 4, -10, 10);
    MagFact.Lat = MagLatGain*JoyCnt_MagLat.Count;

    % - 2) magnitude factor of dorsal joint wave
    JoyCnt_MagDor.CntWithTwoButtons(2, 3, -10, 10);
    MagFact.Dor = MagDorGain*JoyCnt_MagDor.Count;

    
	%% Get the joint angles of SEA snake robot
    %%  - [ Operation mode: Ground Locomotion ]
    if (OperMode == 0)
        
        % Enable/disable the sub-functions,'Head Look' of ground locomotion with joystick input
        EnableSubFcn = GetFlagSubFcn_GroundLocomotion(EnableSubFcn, buttons, buttons_prev);
        
        % Set the direction of the wave using joystick
        dir = GetDirUsingJoy_GroundLocomotion(axes, buttons, povs, bTransDoneJoyReady);

        % Update the internal time of the gait equation with given direction of the wave, 'dir'
        sm.Ot = GetIntTimeOt(dir, sm.Ot, (1+VelFactor)*dt);

        % Get joint angles, SEA_angles
        SEA_angles = GetJointAngles_GroundLocomotion(sm.Ot, snakeData, MagFact, EnableSubFcn, SEA_angles_prev, axes, buttons, povs, dt);
        
    %%  - [ Operation mode: Outer-pipe Inspection ]
    elseif (OperMode == 1)
        
        % Enable/disable the sub-functions of outer pipe inspection with joystick input
        % such as 'Head Look', 'Crawling between pipes', 'Rotating in place', 'Shinfting from Rod to Pipe'
        EnableSubFcn = GetFlagsSubFcn_OuterPipeInspection(fbk, EnableSubFcn, buttons, buttons_prev);       
  
        
        % Set the direction of movement for gaits using joystick
        dir = GetDirUsingJoy_OuterPipeInspection(povs, EnableSubFcn, bTransDoneJoyReady);
        
        % Update the internal time of the gait equation
        sm.Ot = GetIntTimeOt(dir, sm.Ot, (1+VelFactor)*dt);

        
        % Set the direction of movement only for the rotating in place gait using joystick
        %  -> this is required because this gait is executed by stopping the rolling and rotating in place with its own internal time
        dir_RiP = GetDirUsingJoy_RotInPlace_OutPipeInspect(povs, EnableSubFcn, bTransDoneJoyReady);
        
        % Update the internal time only for the rotating in place
        Ot_RiP = GetIntTimeOt(dir_RiP, Ot_RiP, (1+VelFactor)*dt);
        
        
        % Get joint angles, SEA_angles and the snake modulation, sm
        Res = GetJointAngles_SM_OuterPipeInspection(sm, snakeData, fbk, VelFactor, radi, pitch, PoleOnLeft, SEA_angles, EnableSubFcn, Ot_offset_ShiftPipe, axes, buttons, buttons_prev, povs, dt, Ot_RiP);
        SEA_angles = Res.Angles;
        sm = Res.sm;
        Ot_offset_ShiftPipe = Res.Ot_offset_ShiftPipe;
        
    end       
        

	%% Run the transition motion when user changes the gait
    if simQ
        GaitTransMot_Sim();
    else
        GaitTransMot_Exp();
    end

    
	%% Move the robot (update the desired joint angles)
    if simQ
        % update the joint angles 
        plt.plot(SEA_angles); 

        % delaying
        pause(dt);
    else
        % send the position command
        cmd.position = SEA_angles;      % Substitute SEA_angles into the position command, cmd.position
        snake.set(cmd);                 % Send the position command
    end
    
    
	%% Update the previous values
    SEA_angles_prev = SEA_angles;
    
    buttons_prev = buttons;
    
    JoyCnt_Vel.buttons_prev = buttons_prev;
    JoyCnt_MagLat.buttons_prev = buttons_prev;
    JoyCnt_MagDor.buttons_prev = buttons_prev;
    
    
end % end of main code

fprintf('\n')
disp('Exitted.');


%% (experiment) the following stops the data logging and makes the snake go limp:
if ~simQ
    % Stop logging
    if LogData
        log = snake.stopLogFull;
        disp('Data is logged in `log`')
    end
    
    % Go limps
    snake.setCommandLifetime(0);

    cmd.position = nan(1,numModules);
    cmd.torque = zeros(1,numModules);
    snake.set(cmd);
end