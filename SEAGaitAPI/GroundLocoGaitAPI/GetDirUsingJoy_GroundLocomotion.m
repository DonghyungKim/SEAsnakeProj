% This function returns the direction of the wave according to user's joystick input for the normal mode  -  by Donghyung Kim, 16.09.23 (revised)
% [Inputs]
%   - axes, buttons, povs : joystick reading, for example, [axes, buttons, povs] = read( joy );
% [Output]
%   - dir (boolean or empty) : the diretion of the wave. This value can be true/false/[].
%                              Here, 'true' and 'false' are used for the forward and the backward direction of the wave, respectively.
%                              And empty ([]) is to stop the wave, which means the robot stops

function dir = GetDirUsingJoy_GroundLocomotion(axes, buttons, povs, bTransDoneJoyReady)
    %  - No direction of movement until the transition motion is done 
    if ~bTransDoneJoyReady
        dir = [];
        
    %  - Case that the direction of movement becomes 'true'
    elseif ((povs == 0) && ((~buttons(5))&&(~buttons(6))&&(~sign(axes(1)))&&(~sign(axes(3))))) ...   	% D-pad, 0 degree  => go forward / slithering
            ||((povs == 90) && ((~buttons(5))&&(~buttons(6))&&(~sign(axes(1)))&&(~sign(axes(3))))) ...	% D-pad, 90 degree =>  go right  / sidewinding
            ||((sign(axes(1))==1) && ((povs == -1)&&(~buttons(5))&&(~buttons(6))&&(~sign(axes(3))))) ...% Analog Joystick Axis 1, right  =>  turn right / conical sidewinding  
            ||((sign(axes(3))==1) && ((povs == -1)&&(~buttons(5))&&(~buttons(6))&&(~sign(axes(1))))) ...% Analog Joystick Axis 3, right  =>  spin right / turn-in-place 
            ||((buttons(5)) && ((povs == -1)&&(~buttons(6))&&(~sign(axes(1)))&&(~sign(axes(3)))))     	% buton 5 On => roll left / lateral rolling
        dir = true;

    %  - Case that the direction of movement becomes 'false'    
    elseif ((povs == 180) && ((~buttons(5))&&(~buttons(6))&&(~sign(axes(1)))&&(~sign(axes(3))))) ...        % D-pad, 180 degree  => go backward / slithering
            ||((povs == 270) && ((~buttons(5))&&(~buttons(6))&&(~sign(axes(1)))&&(~sign(axes(3)))))...      % D-pad, 270 degree =>  go left  / sidewinding
            ||((sign(axes(1)) == -1) && ((povs == -1)&&(~buttons(5))&&(~buttons(6))&&(~sign(axes(3))))) ... % Analog Joystick Axis 1, left  =>  turn left / conical sidewinding  
            ||((sign(axes(3)) == -1) && ((povs == -1)&&(~buttons(5))&&(~buttons(6))&&(~sign(axes(1))))) ... % Analog Joystick Axis 3, left  =>  spin left / turn-in-place 
            ||((buttons(6)) && ((povs == -1)&&(~buttons(5))&&(~sign(axes(1)))&&(~sign(axes(3)))))           % buton 6 On => roll right / lateral rolling
        dir = false;

    %  - Do nothing with no joystick input
    else
        dir = [];
    end
end