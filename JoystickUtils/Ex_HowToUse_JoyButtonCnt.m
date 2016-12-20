% This example is to show how to use 'JoyButtonCnt' class  
% -  by Donghyung Kim, 2016.08.29

clear

joy           = vrjoystick(1);  % create the joystick object

bTerminate    = false;          % boolean that determines whether the  program is running or terminated.
                                % The on/off button for this program is both buttons 9 and 10 on the controller.

% Declare Joystick Button Counter class
JoyCnt_1_2 = JoyButtonCnt;
JoyCnt_5_6 = JoyButtonCnt;

%% Main code

while ~bTerminate
    %% Joystick reading
    [axes, buttons, povs] = read( joy );    % axes, buttons, povs are the data that the joystick will return
    bTerminate = buttons(9)&buttons(10);     % both buttons 9 & 10 terminate the program
   
    JoyCnt_1_2.Update(axes, buttons, povs);
    JoyCnt_5_6.Update(axes, buttons, povs);
    
    JoyCnt_1_2.CntWithTwoButtons(1, 2, -5, 5);
    JoyCnt_5_6.CntWithTwoButtons(5, 6, -4, 4);
    
    %% Print the number of pushes
    fprintf('JoyCnt_1_2.Count = %d   /   JoyCnt_5_6.Count = %d \n', JoyCnt_1_2.Count, JoyCnt_5_6.Count);
    
    
	%% Update the previous values
	JoyCnt_1_2.buttons_prev = buttons;
    JoyCnt_5_6.buttons_prev = buttons;
     
    pause(0.1)
    
end % end of main code

fprintf('\n')
disp('Exitted.');