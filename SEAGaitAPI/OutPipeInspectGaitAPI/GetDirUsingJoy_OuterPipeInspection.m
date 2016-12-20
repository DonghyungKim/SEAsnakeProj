% This function returns the direction of the wave according to user's joystick input for the outer-pipe Inspection  -  by Donghyung Kim, 16.08.16
% [Inputs]
%   - axes, buttons, povs : joystick reading, for example, [axes, buttons, povs] = read( joy );
% [Output]
%   - dir (boolean or empty) : the diretion of the wave. This value can be true/false/[].
%                              Here, 'true' and 'false' are used for the forward and the backward direction of the wave, respectively.
%                              And empty ([]) is to stop the wave, which means the robot stops

function dir = GetDirUsingJoy_OuterPipeInspection(povs, EnableSubFcn, bTransDoneJoyReady)
	%  - No rolling movement when waiting until the transition motion is done or executing the rotating in place
    if (~bTransDoneJoyReady)||(EnableSubFcn.RotInPlace_OPI)
        dir = [];

	%  - (default) pole climing, crawling between pipes
    elseif (povs == 0)
        dir = false;
    elseif (povs == 180)
        dir = true;
        
    %  - No rolling with no controller input
    else
        dir = [];
    end
    
    %  - reverse direction for the crawling between two pipes
    if EnableSubFcn.PipeCraw_OPI
        dir = ~dir;
    end
end