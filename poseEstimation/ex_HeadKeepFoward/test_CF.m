clear
clc

% converting factor
R2D = 180/pi;   % radian to degree
D2R = pi/180;   % degree to radian


% add the paths
cd D:\2016_Snake_Robot\MATLAB_source_codes\SEAsnakeProj
addpath(genpath(pwd));


% initialize the SEA snake robot
initializeSEASnake();


% load offsets
cd D:\2016_Snake_Robot\MATLAB_source_codes\SEAsnakeProj\poseEstimation\ex_HeadKeepFoward
load('offsets.mat');

% Declare the complementary filter
CF = ComplementaryFilter(snakeData,'accelOffsets',accelOffsets(:,1:numModules),...
       'gyroOffsets',gyroOffsets(:,1:numModules),'gyrosTrustability',...
        gyrosTrustability(1:numModules),'accTrustability',accTrustability(1:numModules));

    
% input the running time
ft = 120;     % Unit: second
    
    
% set feedback rate
fbkHz = 100;    %200;    %300;
snake.setFeedbackFrequency(fbkHz);

Tmpfbk  = snake.getNextFeedback();

% time interval  -> determined by the feedback rate
dt = 1/fbkHz;

% update the object with the current feedback values twice 
fbk = snake.getNextFeedback();
CF.update(fbk);

pause(0.25);

fbk = snake.getNextFeedback();
CF.update(fbk);


% Go limb
cmd = CommandStruct();  % initialize the command structure

snake.setCommandLifetime(0);

cmd.position = nan(1,numModules);
cmd.torque = zeros(1,numModules);
snake.set(cmd);


% get the current time
t = Tmpfbk.time;


%% Main code
cnt = 1;
cnt_max = ft/dt;
while cnt < cnt_max
%     cmd.position = nan(1,numModules);                   % get NaN for the position command
%     cmd.position(numModules) = (30*pi/180)*sin(2*pi*t); % rotates the only head module +-30 degree with 1 second period. Note that the module which has the largest index is the head (SEA snake)
    
    cmd.position = zeros(1, numModules);

    t = Tmpfbk.time;    % update the current time
	snake.set(cmd);     % send the position command
    
    
    fbk = snake.getNextFeedback(Tmpfbk);
    
	CF.update(snake.getNextFeedback());
    
    T_head = CF.getInGravity ('head');
    T_VC = CF.getInGravity ('VC');
    T_tail = CF.getInGravity ('tail');
    T_wholebody = CF.getInGravity ('wholeBody');
    
    x_head = HomoMat_2_vector(T_head);
    x_VC = HomoMat_2_vector(T_VC);
    x_tail = HomoMat_2_vector(T_tail);
    x_wholebody = HomoMat_2_vector(T_wholebody);
    
    for i=1:6
        if i <= 3
            fprintf('%.3f   ', x_head(i))
        else
            fprintf('%.3f   ', x_head(i)*R2D)
        end
    end
    fprintf('\n')
    
    cnt = cnt + 1;
end
disp('Done.')
