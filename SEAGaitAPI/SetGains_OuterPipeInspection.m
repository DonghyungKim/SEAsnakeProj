% Gains for the outer pipe inspection

% set all gains the same
gains = snake.getGains();
ones_n = ones(1,numModules);
gains.controlStrategy = ones_n*4;
%%% PID gains for settng module positions:
gains.positionKp = ones_n*4;    % Proportional gain (important)
gains.positionKi = ones_n*0.01; % Integral gain (should be low)
gains.positionKd = ones_n*1;    % Derivative gain (important)

gains.torqueKp = ones_n*1;
gains.torqueKi = ones_n*0;
gains.torqueKd = ones_n*.1;
gains.torqueMaxOutput = ones_n*2.25;
gains.torqueMinOutput = -gains.torqueMaxOutput;
gains.positionIClamp= ones_n*1;
gains.velocityKp = ones_n*1;
gains.positionMaxOutput = ones_n*10;
gains.positionMinOutput = ones_n*-10;
gains.torqueOutputLowpassGain = ones_n*.5;
gains.torqueFF = ones_n*0.15;
% snake.set('persist', true); % set the gains to persist
snake.set('gains',gains);
%snake.setCommandLifetime(0);
