% This is an example about keeping the head stationary using the
% complimentary filter - by Dong-Hyung Kim, 2016.09.01

clear
clc

%% Initialize this example - running time, file path...

% input the running time
ft = 30;     % Unit: second

% converting factor
R2D = 180/pi;   % radian to degree
D2R = pi/180;   % degree to radian


% add the paths
cd D:\2016_Snake_Robot\MATLAB_source_codes\SEAsnakeProj
addpath(genpath(pwd));

%% Initialize the robot and the control parameters

% initialize the SEA snake robot
initializeSEASnake();

% initialize the command structure
cmd = CommandStruct(); 

% set feedback rate
fbkHz = 100;    %200;    %300;
snake.setFeedbackFrequency(fbkHz);

Tmpfbk  = snake.getNextFeedback();

% time interval  -> determined by the feedback rate
dt = 1/fbkHz;

%% Initialize the CF (complementary filter)

% load offsets
cd D:\2016_Snake_Robot\MATLAB_source_codes\SEAsnakeProj\poseEstimation\ex_HeadKeepFoward
load('offsets.mat');

% Declare the CF
CF = ComplementaryFilter(snakeData,'accelOffsets',accelOffsets(:,1:numModules),...
       'gyroOffsets',gyroOffsets(:,1:numModules),'gyrosTrustability',...
        gyrosTrustability(1:numModules),'accTrustability',accTrustability(1:numModules));

% update the object with the current feedback values twice 
fbk = snake.getNextFeedback();
CF.update(fbk);

pause(0.25);

fbk = snake.getNextFeedback();
CF.update(fbk);


% obtain initial value of x, x_head_init and print it
disp('Initial value for the head frame is ');
T_head = CF.getInGravity ('head');
x_head = HomoMat_2_vector(T_head);
x_head_init = x_head;
for i=1:6
    if i <= 3
        fprintf('%.3f   ', x_head_init(i))
    else
        fprintf('%.3f   ', x_head_init(i)*R2D)
    end
end
fprintf('\n');
disp('--------------------------------------------------------');


% make the robot's shape in straight
cmd.position = zeros(1, numModules);
snake.set(cmd);     % send the position command
pause(0.5);


%% Main code

% get the current time
t = Tmpfbk.time;

% determine the counter and its maximum value for the main loop
cnt = 1;
cnt_max = ft/dt;

while cnt < cnt_max
    
    % Control two modules to keep head's orientation stationary 
	cmd.position = HeadTwoMods_KeepFoward(cmd.position, x_head_init, x_head, snakeData.num_modules);
    
    
    t = Tmpfbk.time;    % update the current time
	snake.set(cmd);     % send the position command
    
    
    % get the feedback
    fbk = snake.getNextFeedback(Tmpfbk);
    
    % Update the complimentary filter / get T_head and x_head
	CF.update(snake.getNextFeedback());
    
    T_head = CF.getInGravity ('head');  % transformation matrix from the gravity frame to the head frame
    x_head =  HomoMat_2_vector(T_head); % 

    
    % print time and x_head
    fprintf('%.3f   ', cnt*dt)
    for i=1:6
        if i <= 3
            fprintf('%.3f   ', x_head(i))
        else
            fprintf('%.3f   ', x_head(i)*R2D)
        end
    end
    fprintf('\n')
    
    
    % increase the counter
    cnt = cnt + 1;
end
disp('Done.')


%% Go limb
snake.setCommandLifetime(0);

cmd.position = nan(1,numModules);
cmd.torque = zeros(1,numModules);
snake.set(cmd);