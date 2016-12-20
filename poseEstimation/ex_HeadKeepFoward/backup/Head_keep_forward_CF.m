% 모델번호 적힌 Head 면을 기준으로 시작한 경우, 로봇을 뒤집어도 잘 작동함
% (반대의 경우 동작 안하는 것 같으며, 검증 필요)

clear
clc

R2D = 180/pi;
D2R = pi/180;

% add the paths
addpath(genpath(pwd));

load('offsets.mat');

snake = HebiLookup.newConnectedGroupFromName('SEA-Snake','SA002');
snakeData = setupSnakeData( 'SEA Snake', snake.getInfo.numModules);
numModules = snake.getInfo.numModules;

% Declare the complementary filter
CF = ComplementaryFilter(snakeData,'accelOffsets',accelOffsets(:,1:numModules),...
       'gyroOffsets',gyroOffsets(:,1:numModules),'gyrosTrustability',...
        gyrosTrustability(1:numModules),'accTrustability',accTrustability(1:numModules));

    
% input the running time
ft = 60;     % Unit: second
    
    
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



%
disp('Initial value for the head frame is ');
T_head = CF.getInGravity ('head');
x_head = HomoMat_2_vector(T_head);
x_head_init = x_head

disp('Initial value for the tail frame is ');
T_tail = CF.getInGravity ('tail');
x_tail = HomoMat_2_vector(T_tail);
x_tail_init = x_tail

disp('--------------------------------------------------------');

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

cmd.position = zeros(1, numModules);
while cnt < cnt_max
%     cmd.position = nan(1,numModules);                   % get NaN for the position command
%     cmd.position(numModules) = (30*pi/180)*sin(2*pi*t); % rotates the only head module +-30 degree with 1 second period. Note that the module which has the largest index is the head (SEA snake)


    if ((x_head(4) > -110*D2R) && (x_head(4) < -70*D2R))  % LED만 있는 면 (모델번호 없음)이 위로 온 경우
        head_comp_alpha = 2.0*(x_head(4) + 90*D2R);
        cmd.position(16) = head_comp_alpha;
        
        if (x_head(6) > pi/2) && (x_head(6) < pi)
            head_comp_gamma = 1.2*(pi - x_head(6) - x_head_init(6));
            cmd.position(15) = head_comp_gamma;
        elseif (x_head(6) < -pi/2) && (x_head(6) > -pi)  % ex) -173 deg             %it work
            head_comp_gamma = 1.2*(pi + x_head(6) - x_head_init(6));
            cmd.position(15) = -head_comp_gamma;

        end
        
    elseif ((x_head(4) > 70*D2R) && (x_head(4) < 110*D2R)) % 모델번호 적힌 면이 위로 온 경우 % It work
        head_comp_alpha = 2.0*(x_head(4) - 90*D2R);
        head_comp_gamma = 1.2*(x_head(6) - x_head_init(6));
        
        cmd.position(16) = head_comp_alpha;
        cmd.position(15) = head_comp_gamma;
    end
    
    
    
    
    % 	if (head_comp_alpha < -pi)
% 		head_comp_alpha = head_comp_alpha + 2.0*pi; % Force the orientation into (-pi,pi)
%     elseif (head_comp_alpha >= pi)
% 		head_comp_alpha = head_comp_alpha - 2.0*pi;
%     end
    
    

%     % Head
%     if (x_head(4) < 0) 
%         head_comp_alpha = 2.0*(x_head(4) - x_head_init(4)); %x_head(4) - x_head_init(4);
%     else
%         head_comp_alpha = 2.0*(-x_head(4) - x_head_init(4));
%     end
%         
% 	if (head_comp_alpha < -pi)
% 		head_comp_alpha = head_comp_alpha + 2.0*pi; % Force the orientation into (-pi,pi)
%     elseif (head_comp_alpha >= pi)
% 		head_comp_alpha = head_comp_alpha - 2.0*pi;
%     end
%     
%     head_comp_gamma = 2.0*(x_head(6) - x_head_init(6)); %x_head(4) - x_head_init(4);
    
% 	if (head_comp_gamma < -pi)
% 		head_comp_gamma = head_comp_gamma + 2.0*pi; % Force the orientation into (-pi,pi)
%     elseif (head_comp_gamma >= pi)
% 		head_comp_gamma = head_comp_gamma - 2.0*pi;
%     end
    
    

%     % Head position commands
%     cmd.position(16) = head_comp_alpha;
% 	cmd.position(15) = head_comp_gamma;
    
    
%     % Tail
% 	tail_comp_alpha = 2.5*(x_tail(4) - x_tail_init(4));
%     
% 	if (tail_comp_alpha < -pi)
% 		tail_comp_alpha = tail_comp_alpha + 2.0*pi; % Force the orientation into (-pi,pi)
%     elseif (tail_comp_alpha >= pi)
% 		tail_comp_alpha = tail_comp_alpha - 2.0*pi;
%     end   
    
    % Tail position commands
	%cmd.position(2) = head_comp_alpha;
    
    
    
    

    t = Tmpfbk.time;    % update the current time
	snake.set(cmd);     % send the position command
    
    
    fbk = snake.getNextFeedback(Tmpfbk);
    
	CF.update(snake.getNextFeedback());
    
    T_head = CF.getInGravity ('head');
% 	T_VC = CF.getInGravity ('VC');
	T_tail = CF.getInGravity ('tail');
%     T_wholebody = CF.getInGravity ('wholeBody');
    
    x_head =  HomoMat_2_vector(T_head);
% 	x_VC = HomoMat_2_vector(T_VC);
	x_tail = HomoMat_2_vector(T_tail);
%     x_wholebody = HomoMat_2_vector(T_wholebody);
    
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
