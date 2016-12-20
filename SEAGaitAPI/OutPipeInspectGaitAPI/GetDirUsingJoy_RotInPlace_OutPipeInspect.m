% This function returns the direction of the wave according to user's joystick input for the outer-pipe Inspection  -  by Donghyung Kim, 16.08.16
% [Inputs]
%   - axes, buttons, povs : joystick reading, for example, [axes, buttons, povs] = read( joy );
% [Output]
%   - dir (boolean or empty) : the diretion of the wave. This value can be true/false/[].
%                              Here, 'true' and 'false' are used for the forward and the backward direction of the wave, respectively.
%                              And empty ([]) is to stop the wave, which means the robot stops

function dir = GetDirUsingJoy_RotInPlace_OutPipeInspect(povs, EnableSubFcn, bTransDoneJoyReady)
	%  - No spining movement when waiting until the transition motion is done
    if (~bTransDoneJoyReady)
        dir = [];
        
	%  - Only for the rotating in place
    elseif EnableSubFcn.RotInPlace_OPI
        if (povs == 90)
            dir = false;
        elseif (povs == 270)
            dir = true;
        else
            dir = [];
        end
        
    %  - No spining without controller input
    else
        dir = [];
    end
    
end